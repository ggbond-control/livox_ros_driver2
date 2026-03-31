#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace livox_ros {

using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;

class LivoxMergeNode final : public rclcpp::Node {
 public:
  LivoxMergeNode() : Node("livox_merge_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar");
    publish_freq_ = declare_parameter<double>("publish_freq", 10.0);
    expected_lidars_ = declare_parameter<int>("expected_lidars", 4);
    output_point_cap_ = declare_parameter<int>("output_point_cap", 24000);

    if (publish_freq_ < 0.1) {
      publish_freq_ = 10.0;
    }
    window_ns_ = static_cast<uint64_t>(1e9 / publish_freq_);

    publisher_ = create_publisher<CustomMsg>(output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<CustomMsg>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LivoxMergeNode::OnCustomMsg, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::nanoseconds(window_ns_),
        std::bind(&LivoxMergeNode::OnTimer, this));
  }

 private:
  void OnCustomMsg(const CustomMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    pending_msgs_.push_back(*msg);
  }

  void OnTimer() {
    std::vector<CustomMsg> msgs_to_merge;
    {
      std::lock_guard<std::mutex> lock(msg_mutex_);
      if (pending_msgs_.empty()) {
        return;
      }
      msgs_to_merge.swap(pending_msgs_);
    }

    if (msgs_to_merge.empty()) {
      return;
    }

    CustomMsg merged;
    merged.header.frame_id = msgs_to_merge.front().header.frame_id;
    merged.lidar_id = 0;
    merged.rsvd = {0, 0, 0};

    uint64_t merged_timebase = std::numeric_limits<uint64_t>::max();
    size_t total_points = 0;
    for (const auto &entry : msgs_to_merge) {
      merged_timebase = std::min(merged_timebase, entry.timebase);
      total_points += entry.points.size();
    }

    merged.timebase = merged_timebase;
    merged.header.stamp = rclcpp::Time(merged_timebase);
    merged.points.reserve(total_points);

    for (const auto &entry : msgs_to_merge) {
      const uint64_t base_delta = entry.timebase - merged_timebase;
      for (const auto &pt : entry.points) {
        CustomPoint merged_pt = pt;
        merged_pt.offset_time = static_cast<uint32_t>(base_delta + pt.offset_time);
        merged.points.push_back(std::move(merged_pt));
      }
    }

    std::sort(merged.points.begin(), merged.points.end(),
        [](const CustomPoint &lhs, const CustomPoint &rhs) {
          return lhs.offset_time < rhs.offset_time;
        });

    if (output_point_cap_ > 0 && static_cast<int>(merged.points.size()) > output_point_cap_) {
      const size_t src_size = merged.points.size();
      const size_t dst_size = static_cast<size_t>(output_point_cap_);
      std::vector<CustomPoint> downsampled;
      downsampled.reserve(dst_size);
      for (size_t i = 0; i < dst_size; ++i) {
        const size_t idx = (i * src_size) / dst_size;
        downsampled.push_back(std::move(merged.points[idx]));
      }
      merged.points.swap(downsampled);
    }

    merged.point_num = static_cast<uint32_t>(merged.points.size());

    publisher_->publish(merged);
  }

  std::string input_topic_;
  std::string output_topic_;
  double publish_freq_ = 10.0;
  int expected_lidars_ = 4;
  int output_point_cap_ = 24000;
  uint64_t window_ns_ = 100000000;

  std::mutex msg_mutex_;
  std::vector<CustomMsg> pending_msgs_;
  rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<CustomMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace livox_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<livox_ros::LivoxMergeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
