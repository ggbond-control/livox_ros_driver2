#include <algorithm>
#include <array>
#include <fstream>
#include <future>
#include <iostream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "comm/comm.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

namespace livox_ros {

namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using ColorPointT = pcl::PointXYZRGB;
using ColorCloudT = pcl::PointCloud<ColorPointT>;

struct Pose6d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct LidarRegistrationResult {
  std::string topic;
  std::string ip;
  uint32_t handle = 0;
  bool success = false;
  double fitness = 0.0;
  size_t source_points = 0;
  size_t target_points = 0;
  Eigen::Matrix4f map_to_base_estimate = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_to_lidar_initial = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_to_lidar_refined = Eigen::Matrix4f::Identity();
  builtin_interfaces::msg::Time first_stamp;
  builtin_interfaces::msg::Time last_stamp;
  size_t accumulated_frames = 0;
  CloudT::Ptr raw_accumulated_source_cloud;
};

std::string TopicToIp(const std::string &topic) {
  const auto pos = topic.find("lidar_");
  if (pos == std::string::npos) {
    return "";
  }
  std::string ip = topic.substr(pos + 6);
  std::replace(ip.begin(), ip.end(), '_', '.');
  return ip;
}

Pose6d MatrixToPose(const Eigen::Matrix4f &matrix) {
  Pose6d pose;
  pose.x = matrix(0, 3);
  pose.y = matrix(1, 3);
  pose.z = matrix(2, 3);

  const Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
  const double sy = std::sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
  const bool singular = sy < 1e-6;

  if (!singular) {
    pose.roll = std::atan2(rotation(2, 1), rotation(2, 2));
    pose.pitch = std::atan2(-rotation(2, 0), sy);
    pose.yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    pose.roll = std::atan2(-rotation(1, 2), rotation(1, 1));
    pose.pitch = std::atan2(-rotation(2, 0), sy);
    pose.yaw = 0.0;
  }
  return pose;
}

Eigen::Matrix4f PoseToMatrix(const Pose6d &pose) {
  Eigen::AngleAxisf roll_angle(static_cast<float>(pose.roll), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(static_cast<float>(pose.pitch), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(static_cast<float>(pose.yaw), Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
  transform(0, 3) = static_cast<float>(pose.x);
  transform(1, 3) = static_cast<float>(pose.y);
  transform(2, 3) = static_cast<float>(pose.z);
  return transform;
}

std::string PoseToString(const Pose6d &pose, double trans_scale = 1.0, double angle_scale = 1.0) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "x=" << pose.x * trans_scale
      << ", y=" << pose.y * trans_scale
      << ", z=" << pose.z * trans_scale
      << ", roll=" << pose.roll * angle_scale
      << ", pitch=" << pose.pitch * angle_scale
      << ", yaw=" << pose.yaw * angle_scale;
  return oss.str();
}

CloudT::Ptr DownsampleCloud(const CloudT::Ptr &cloud, double leaf_size) {
  if (!cloud || cloud->empty() || leaf_size <= 0.0) {
    return cloud;
  }
  auto downsampled = std::make_shared<CloudT>();
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(static_cast<float>(leaf_size),
                           static_cast<float>(leaf_size),
                           static_cast<float>(leaf_size));
  voxel_filter.filter(*downsampled);
  return downsampled;
}

Eigen::Matrix4f BuildBaseToLidarMatrix(const UserLivoxLidarConfig &config) {
  Pose6d pose;
  pose.x = static_cast<double>(config.extrinsic_param.x) / 1000.0;
  pose.y = static_cast<double>(config.extrinsic_param.y) / 1000.0;
  pose.z = static_cast<double>(config.extrinsic_param.z) / 1000.0;
  pose.roll = static_cast<double>(config.extrinsic_param.roll) * M_PI / 180.0;
  pose.pitch = static_cast<double>(config.extrinsic_param.pitch) * M_PI / 180.0;
  pose.yaw = static_cast<double>(config.extrinsic_param.yaw) * M_PI / 180.0;
  return PoseToMatrix(pose);
}

bool EnsureDirectory(const std::string &path) {
  if (path.empty()) {
    return false;
  }
  struct stat st = {};
  if (stat(path.c_str(), &st) == 0) {
    return S_ISDIR(st.st_mode);
  }
  return mkdir(path.c_str(), 0755) == 0;
}

std::array<uint8_t, 3> GetColorForIndex(size_t index) {
  static const std::array<std::array<uint8_t, 3>, 4> kColors = {{
      {{255, 64, 64}},
      {{64, 200, 255}},
      {{64, 255, 128}},
      {{255, 196, 64}},
  }};
  return kColors[index % kColors.size()];
}

}  // namespace

class MultiLidarMapCalibratorNode : public rclcpp::Node {
 public:
  MultiLidarMapCalibratorNode()
      : Node("multi_lidar_map_calibrator"),
        map_cloud_(std::make_shared<CloudT>()) {
    declare_parameter<std::vector<std::string>>("lidar_topics", {
        "/livox/lidar_192_168_2_202",
        "/livox/lidar_192_168_2_203",
        "/livox/lidar_192_168_2_204",
        "/livox/lidar_192_168_2_205",
    });
    declare_parameter<std::string>("config_path", "");
    declare_parameter<std::string>("map_pcd_path", "");
    declare_parameter<std::string>("output_path", "");
    declare_parameter<std::string>("debug_pcd_dir", "/tmp/multi_lidar_map_calibrator_debug");
    declare_parameter<bool>("save_debug_pcd", true);
    declare_parameter<std::vector<double>>("initial_base_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<int>("required_accumulated_frames", 20);
    declare_parameter<int>("status_log_interval_frames", 5);
    declare_parameter<double>("source_voxel_size", 0.05);
    declare_parameter<double>("target_voxel_size", 0.08);
    declare_parameter<double>("registration_source_voxel_size", 0.12);
    declare_parameter<double>("registration_target_voxel_size", 0.12);
    declare_parameter<double>("crop_half_extent_xy", 6.0);
    declare_parameter<double>("crop_half_extent_z", 2.5);
    declare_parameter<double>("max_correspondence_distance", 1.0);
    declare_parameter<double>("transformation_epsilon", 1e-4);
    declare_parameter<double>("fitness_epsilon", 1e-3);
    declare_parameter<int>("maximum_iterations", 80);
    declare_parameter<int>("registration_threads", 4);
    declare_parameter<bool>("run_once", true);

    lidar_topics_ = get_parameter("lidar_topics").as_string_array();
    config_path_ = get_parameter("config_path").as_string();
    map_pcd_path_ = get_parameter("map_pcd_path").as_string();
    output_path_ = get_parameter("output_path").as_string();
    debug_pcd_dir_ = get_parameter("debug_pcd_dir").as_string();
    save_debug_pcd_ = get_parameter("save_debug_pcd").as_bool();
    required_accumulated_frames_ = get_parameter("required_accumulated_frames").as_int();
    status_log_interval_frames_ = get_parameter("status_log_interval_frames").as_int();
    source_voxel_size_ = get_parameter("source_voxel_size").as_double();
    target_voxel_size_ = get_parameter("target_voxel_size").as_double();
    registration_source_voxel_size_ = get_parameter("registration_source_voxel_size").as_double();
    registration_target_voxel_size_ = get_parameter("registration_target_voxel_size").as_double();
    crop_half_extent_xy_ = get_parameter("crop_half_extent_xy").as_double();
    crop_half_extent_z_ = get_parameter("crop_half_extent_z").as_double();
    max_correspondence_distance_ = get_parameter("max_correspondence_distance").as_double();
    transformation_epsilon_ = get_parameter("transformation_epsilon").as_double();
    fitness_epsilon_ = get_parameter("fitness_epsilon").as_double();
    maximum_iterations_ = get_parameter("maximum_iterations").as_int();
    registration_threads_ = get_parameter("registration_threads").as_int();
    run_once_ = get_parameter("run_once").as_bool();

    if (!LoadConfigExtrinsics()) {
      throw std::runtime_error("failed to load lidar config extrinsics");
    }
    if (!LoadInitialBasePose()) {
      throw std::runtime_error("failed to parse initial_base_pose");
    }
    if (!LoadMapCloud()) {
      throw std::runtime_error("failed to load map PCD");
    }

    const auto qos = rclcpp::SensorDataQoS();
    for (const auto &topic : lidar_topics_) {
      auto subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, qos,
          [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            OnPointCloud(topic, std::move(msg));
          });
      subscriptions_.push_back(subscription);
      RCLCPP_INFO(get_logger(), "Subscribed to %s", topic.c_str());
    }

    trigger_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MultiLidarMapCalibratorNode::TryCalibrate, this));
  }

 private:
  bool LoadConfigExtrinsics() {
    if (config_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter config_path is empty.");
      return false;
    }
    LivoxLidarConfigParser parser(config_path_);
    std::vector<UserLivoxLidarConfig> configs;
    if (!parser.Parse(configs)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse config file: %s", config_path_.c_str());
      return false;
    }
    for (const auto &config : configs) {
      config_by_ip_[IpNumToString(config.handle)] = config;
    }
    return true;
  }

  bool LoadInitialBasePose() {
    const auto values = get_parameter("initial_base_pose").as_double_array();
    if (values.size() != 6) {
      RCLCPP_ERROR(get_logger(),
                   "initial_base_pose must contain 6 doubles, got %zu.",
                   values.size());
      return false;
    }

    initial_base_pose_.x = values[0];
    initial_base_pose_.y = values[1];
    initial_base_pose_.z = values[2];
    initial_base_pose_.roll = values[3];
    initial_base_pose_.pitch = values[4];
    initial_base_pose_.yaw = values[5];

    RCLCPP_INFO(get_logger(), "Initial map->base pose (m/deg): %s",
                PoseToString(initial_base_pose_, 1.0, 180.0 / M_PI).c_str());
    return true;
  }

  bool LoadMapCloud() {
    if (map_pcd_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter map_pcd_path is empty.");
      return false;
    }
    if (pcl::io::loadPCDFile<PointT>(map_pcd_path_, *map_cloud_) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load map PCD: %s", map_pcd_path_.c_str());
      return false;
    }
    map_cloud_ = DownsampleCloud(map_cloud_, target_voxel_size_);
    RCLCPP_INFO(get_logger(), "Loaded map cloud with %zu points from %s",
                map_cloud_->size(), map_pcd_path_.c_str());
    return true;
  }

  void OnPointCloud(const std::string &topic, sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(*msg, *cloud);
    cloud = DownsampleCloud(cloud, source_voxel_size_);

    std::lock_guard<std::mutex> lock(data_mutex_);
    auto &accumulated_cloud = accumulated_cloud_by_topic_[topic];
    if (!accumulated_cloud) {
      accumulated_cloud = std::make_shared<CloudT>();
    }
    if (accumulated_cloud->empty()) {
      first_stamp_by_topic_[topic] = msg->header.stamp;
    }
    *accumulated_cloud += *cloud;
    latest_cloud_by_topic_[topic] = cloud;
    latest_stamp_by_topic_[topic] = msg->header.stamp;
    accumulated_frames_by_topic_[topic] += 1;

    if (status_log_interval_frames_ > 0 &&
        (accumulated_frames_by_topic_[topic] % static_cast<size_t>(status_log_interval_frames_) == 0)) {
      RCLCPP_INFO(get_logger(),
                  "[%s] accumulated_frames=%zu accumulated_points=%zu",
                  topic.c_str(),
                  accumulated_frames_by_topic_[topic],
                  accumulated_cloud->size());
    }
  }

  bool AllCloudsReady() const {
    for (const auto &topic : lidar_topics_) {
      const auto cloud_it = accumulated_cloud_by_topic_.find(topic);
      const auto frame_it = accumulated_frames_by_topic_.find(topic);
      if (cloud_it == accumulated_cloud_by_topic_.end() || !cloud_it->second || cloud_it->second->empty()) {
        return false;
      }
      if (frame_it == accumulated_frames_by_topic_.end() ||
          frame_it->second < static_cast<size_t>(required_accumulated_frames_)) {
        return false;
      }
    }
    return true;
  }

  void TryCalibrate() {
    if (calibration_running_ || calibration_done_) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!AllCloudsReady()) {
        return;
      }
      calibration_running_ = true;
    }

    auto results = RunCalibration();
    ReportResults(results);
    if (!output_path_.empty()) {
      WriteResults(results);
    }

    calibration_done_ = run_once_;
    calibration_running_ = false;
  }

  std::vector<LidarRegistrationResult> RunCalibration() {
    std::unordered_map<std::string, CloudT::Ptr> clouds_copy;
    std::unordered_map<std::string, size_t> frames_copy;
    std::unordered_map<std::string, builtin_interfaces::msg::Time> first_stamp_copy;
    std::unordered_map<std::string, builtin_interfaces::msg::Time> last_stamp_copy;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      clouds_copy = accumulated_cloud_by_topic_;
      frames_copy = accumulated_frames_by_topic_;
      first_stamp_copy = first_stamp_by_topic_;
      last_stamp_copy = latest_stamp_by_topic_;
    }

    const Eigen::Matrix4f initial_map_to_base = PoseToMatrix(initial_base_pose_);
    const size_t max_parallel_jobs = std::max<size_t>(
        1, std::min<size_t>(lidar_topics_.size(),
                            registration_threads_ > 0
                                ? static_cast<size_t>(registration_threads_)
                                : std::max<size_t>(1, std::thread::hardware_concurrency())));

    auto process_topic = [this, &clouds_copy, &frames_copy, &first_stamp_copy,
                          &last_stamp_copy, &initial_map_to_base](const std::string &topic) {
      LidarRegistrationResult result;
      result.topic = topic;
      result.ip = TopicToIp(topic);
      const auto config_it = config_by_ip_.find(result.ip);
      if (config_it == config_by_ip_.end()) {
        RCLCPP_ERROR(get_logger(), "No config entry found for lidar ip %s", result.ip.c_str());
        return result;
      }
      result.handle = config_it->second.handle;
      result.base_to_lidar_initial = BuildBaseToLidarMatrix(config_it->second);
      result.accumulated_frames = frames_copy[topic];
      result.first_stamp = first_stamp_copy[topic];
      result.last_stamp = last_stamp_copy[topic];

      const auto cloud_it = clouds_copy.find(topic);
      if (cloud_it == clouds_copy.end() || !cloud_it->second || cloud_it->second->empty()) {
        RCLCPP_WARN(get_logger(), "No cloud data available for %s", topic.c_str());
        return result;
      }

      const auto raw_source_cloud = cloud_it->second;
      result.raw_accumulated_source_cloud = raw_source_cloud;
      auto source_cloud = DownsampleCloud(raw_source_cloud, registration_source_voxel_size_);
      auto target_cloud = DownsampleCloud(map_cloud_, registration_target_voxel_size_);

      RCLCPP_INFO(
          get_logger(),
          "[%s] registration input: accumulated_frames=%zu raw_source_points=%zu registration_source_points=%zu target_points=%zu",
          topic.c_str(),
          result.accumulated_frames,
          raw_source_cloud ? raw_source_cloud->size() : 0,
          source_cloud ? source_cloud->size() : 0,
          target_cloud ? target_cloud->size() : 0);

      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setInputSource(source_cloud);
      gicp.setInputTarget(target_cloud);
      gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);
      gicp.setTransformationEpsilon(transformation_epsilon_);
      gicp.setEuclideanFitnessEpsilon(fitness_epsilon_);
      gicp.setMaximumIterations(maximum_iterations_);

      CloudT aligned_cloud;
      const Eigen::Matrix4f initial_guess = initial_map_to_base;
      gicp.align(aligned_cloud, initial_guess);

      result.success = gicp.hasConverged();
      result.fitness = gicp.getFitnessScore();
      result.source_points = source_cloud->size();
      result.target_points = target_cloud->size();
      result.map_to_base_estimate = gicp.getFinalTransformation();
      RCLCPP_INFO(get_logger(), "[%s] registration finished: success=%s fitness=%.6f",
                  topic.c_str(), result.success ? "true" : "false", result.fitness);
      return result;
    };

    std::vector<LidarRegistrationResult> results;
    results.reserve(lidar_topics_.size());
    std::vector<std::future<LidarRegistrationResult>> futures;
    futures.reserve(max_parallel_jobs);
    size_t next_topic_index = 0;

    auto collect_one = [&results, &futures]() {
      results.push_back(futures.front().get());
      futures.erase(futures.begin());
    };

    while (next_topic_index < lidar_topics_.size() || !futures.empty()) {
      while (next_topic_index < lidar_topics_.size() && futures.size() < max_parallel_jobs) {
        futures.push_back(std::async(std::launch::async, process_topic, lidar_topics_[next_topic_index]));
        ++next_topic_index;
      }
      if (!futures.empty()) {
        collect_one();
      }
    }

    for (auto &result : results) {
      if (!result.success) {
        continue;
      }
      const Eigen::Matrix4f correction_delta = initial_map_to_base.inverse() * result.map_to_base_estimate;
      result.base_to_lidar_refined = correction_delta * result.base_to_lidar_initial;
    }

    return results;
  }

  void ReportResults(const std::vector<LidarRegistrationResult> &results) {
    RCLCPP_INFO(get_logger(), "========== Multi-LiDAR map calibration ==========");

    std::ostringstream json_output;
    json_output << "{\n";
    json_output << "  \"lidar_configs\": [\n";
    bool first_json_item = true;

    for (const auto &result : results) {
      if (!result.success) {
        RCLCPP_WARN(get_logger(), "[%s] registration failed", result.topic.c_str());
        continue;
      }

      const Pose6d map_pose = MatrixToPose(result.map_to_base_estimate);
      const Pose6d initial_pose = MatrixToPose(result.base_to_lidar_initial);
      const Pose6d refined_pose = MatrixToPose(result.base_to_lidar_refined);
      const Eigen::Matrix4f delta = result.base_to_lidar_initial.inverse() * result.base_to_lidar_refined;
      const Pose6d delta_pose = MatrixToPose(delta);

      RCLCPP_INFO(
          get_logger(),
          "[%s] fitness=%.6f source_points=%zu target_points=%zu accumulated_frames=%zu stamp=[%d.%09u -> %d.%09u]",
          result.topic.c_str(), result.fitness, result.source_points, result.target_points,
          result.accumulated_frames,
          result.first_stamp.sec, result.first_stamp.nanosec,
          result.last_stamp.sec, result.last_stamp.nanosec);
      RCLCPP_INFO(get_logger(), "  map->base estimate : %s", PoseToString(map_pose, 1.0, 180.0 / M_PI).c_str());
      RCLCPP_INFO(get_logger(), "  base->lidar initial (m/deg): %s",
                  PoseToString(initial_pose, 1.0, 180.0 / M_PI).c_str());
      RCLCPP_INFO(get_logger(), "  base->lidar refined (m/deg): %s",
                  PoseToString(refined_pose, 1.0, 180.0 / M_PI).c_str());
      RCLCPP_INFO(get_logger(), "  delta initial->refined (m/deg): %s",
                  PoseToString(delta_pose, 1.0, 180.0 / M_PI).c_str());

      if (!first_json_item) {
        json_output << ",\n";
      }
      first_json_item = false;
      json_output << "    {\n";
      json_output << "      \"ip\": \"" << result.ip << "\",\n";
      json_output << "      \"pcl_data_type\": 1,\n";
      json_output << "      \"pattern_mode\": 0,\n";
      json_output << "      \"extrinsic_parameter\": {\n";
      json_output << std::fixed << std::setprecision(6);
      json_output << "        \"roll\": " << refined_pose.roll * 180.0 / M_PI << ",\n";
      json_output << "        \"pitch\": " << refined_pose.pitch * 180.0 / M_PI << ",\n";
      json_output << "        \"yaw\": " << refined_pose.yaw * 180.0 / M_PI << ",\n";
      json_output << "        \"x\": " << refined_pose.x * 1000.0 << ",\n";
      json_output << "        \"y\": " << refined_pose.y * 1000.0 << ",\n";
      json_output << "        \"z\": " << refined_pose.z * 1000.0 << "\n";
      json_output << "      }\n";
      json_output << "    }";
    }

    json_output << "\n";
    json_output << "  ]\n";
    json_output << "}";
    RCLCPP_INFO(get_logger(), "Refined lidar_configs JSON block:\n%s", json_output.str().c_str());

    if (save_debug_pcd_) {
      SaveDebugPcds(results);
    }
  }

  void SaveDebugPcds(const std::vector<LidarRegistrationResult> &results) {
    if (debug_pcd_dir_.empty()) {
      RCLCPP_WARN(get_logger(), "save_debug_pcd is enabled but debug_pcd_dir is empty.");
      return;
    }
    if (!EnsureDirectory(debug_pcd_dir_)) {
      RCLCPP_ERROR(get_logger(), "Failed to create debug pcd directory: %s", debug_pcd_dir_.c_str());
      return;
    }

    auto merged_current = std::make_shared<ColorCloudT>();
    auto merged_refined = std::make_shared<ColorCloudT>();

    for (size_t i = 0; i < results.size(); ++i) {
      const auto &result = results[i];
      if (!result.success || !result.raw_accumulated_source_cloud || result.raw_accumulated_source_cloud->empty()) {
        continue;
      }

      const auto color = GetColorForIndex(i);
      const Eigen::Matrix4f correction_delta =
          result.base_to_lidar_refined * result.base_to_lidar_initial.inverse();

      CloudT refined_cloud;
      pcl::transformPointCloud(*result.raw_accumulated_source_cloud, refined_cloud, correction_delta);

      merged_current->reserve(merged_current->size() + result.raw_accumulated_source_cloud->size());
      for (const auto &pt : result.raw_accumulated_source_cloud->points) {
        ColorPointT out;
        out.x = pt.x;
        out.y = pt.y;
        out.z = pt.z;
        out.r = color[0];
        out.g = color[1];
        out.b = color[2];
        merged_current->push_back(out);
      }

      merged_refined->reserve(merged_refined->size() + refined_cloud.size());
      for (const auto &pt : refined_cloud.points) {
        ColorPointT out;
        out.x = pt.x;
        out.y = pt.y;
        out.z = pt.z;
        out.r = color[0];
        out.g = color[1];
        out.b = color[2];
        merged_refined->push_back(out);
      }
    }

    merged_current->width = static_cast<uint32_t>(merged_current->size());
    merged_current->height = 1;
    merged_current->is_dense = false;
    merged_refined->width = static_cast<uint32_t>(merged_refined->size());
    merged_refined->height = 1;
    merged_refined->is_dense = false;

    const std::string current_path =
        debug_pcd_dir_ + "/merged_current_json_colored.pcd";
    const std::string refined_path =
        debug_pcd_dir_ + "/merged_calibrated_json_colored.pcd";

    pcl::io::savePCDFileBinary(current_path, *merged_current);
    pcl::io::savePCDFileBinary(refined_path, *merged_refined);

    RCLCPP_INFO(
        get_logger(),
        "Saved merged debug PCDs to %s: merged_current_json_colored.pcd and merged_calibrated_json_colored.pcd",
        debug_pcd_dir_.c_str());
  }

  void WriteResults(const std::vector<LidarRegistrationResult> &results) {
    std::ofstream output(output_path_);
    if (!output.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open output file: %s", output_path_.c_str());
      return;
    }

    output << "# multi lidar calibration result\n";
    for (const auto &result : results) {
      if (!result.success) {
        continue;
      }
      const Pose6d refined_pose = MatrixToPose(result.base_to_lidar_refined);
      output << "- topic: " << result.topic << "\n";
      output << "  ip: " << result.ip << "\n";
      output << "  fitness: " << result.fitness << "\n";
      output << "  accumulated_frames: " << result.accumulated_frames << "\n";
      output << "  refined_extrinsic:\n";
      output << "    x_m: " << refined_pose.x << "\n";
      output << "    y_m: " << refined_pose.y << "\n";
      output << "    z_m: " << refined_pose.z << "\n";
      output << "    roll_deg: " << refined_pose.roll * 180.0 / M_PI << "\n";
      output << "    pitch_deg: " << refined_pose.pitch * 180.0 / M_PI << "\n";
      output << "    yaw_deg: " << refined_pose.yaw * 180.0 / M_PI << "\n";
    }
    RCLCPP_INFO(get_logger(), "Wrote calibration result to %s", output_path_.c_str());
  }

  std::vector<std::string> lidar_topics_;
  std::string config_path_;
  std::string map_pcd_path_;
  std::string output_path_;
  std::string debug_pcd_dir_;
  bool save_debug_pcd_ = true;
  Pose6d initial_base_pose_;
  int required_accumulated_frames_ = 20;
  int status_log_interval_frames_ = 5;
  double source_voxel_size_ = 0.05;
  double target_voxel_size_ = 0.08;
  double registration_source_voxel_size_ = 0.12;
  double registration_target_voxel_size_ = 0.12;
  double crop_half_extent_xy_ = 6.0;
  double crop_half_extent_z_ = 2.5;
  double max_correspondence_distance_ = 1.0;
  double transformation_epsilon_ = 1e-4;
  double fitness_epsilon_ = 1e-3;
  int maximum_iterations_ = 80;
  int registration_threads_ = 4;
  bool run_once_ = true;

  CloudT::Ptr map_cloud_;
  std::unordered_map<std::string, UserLivoxLidarConfig> config_by_ip_;
  std::unordered_map<std::string, CloudT::Ptr> accumulated_cloud_by_topic_;
  std::unordered_map<std::string, size_t> accumulated_frames_by_topic_;
  std::unordered_map<std::string, builtin_interfaces::msg::Time> first_stamp_by_topic_;
  std::unordered_map<std::string, CloudT::Ptr> latest_cloud_by_topic_;
  std::unordered_map<std::string, builtin_interfaces::msg::Time> latest_stamp_by_topic_;

  std::mutex data_mutex_;
  bool calibration_running_ = false;
  bool calibration_done_ = false;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr trigger_timer_;
};

}  // namespace livox_ros

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<livox_ros::MultiLidarMapCalibratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[multi_lidar_map_calibrator] fatal: " << e.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
