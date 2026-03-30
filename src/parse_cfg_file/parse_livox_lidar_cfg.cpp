//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "parse_livox_lidar_cfg.h"
#include <iostream>
#include <cmath>

namespace livox_ros {

namespace {

int32_t GetJsonInt32(const rapidjson::Value &value, const char *member_name, int32_t default_value) {
  if (!value.HasMember(member_name)) {
    return default_value;
  }

  const auto &member = value[member_name];
  if (member.IsInt()) {
    return static_cast<int32_t>(member.GetInt());
  }
  if (member.IsUint()) {
    return static_cast<int32_t>(member.GetUint());
  }
  if (member.IsNumber()) {
    return static_cast<int32_t>(std::lround(member.GetDouble()));
  }

  return default_value;
}

float GetJsonFloat(const rapidjson::Value &value, const char *member_name, float default_value) {
  if (!value.HasMember(member_name)) {
    return default_value;
  }

  const auto &member = value[member_name];
  if (member.IsNumber()) {
    return static_cast<float>(member.GetDouble());
  }

  return default_value;
}

}  // namespace

bool LivoxLidarConfigParser::Parse(std::vector<UserLivoxLidarConfig> &lidar_configs) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    std::cout << "failed to open config file: " << path_ << std::endl;
    return false;
  }

  lidar_configs.clear();
  char read_buffer[kMaxBufferSize];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  rapidjson::Document doc;

  do {
    if (doc.ParseStream(config_file).HasParseError()) {
      std::cout << "failed to parse config jason" << std::endl;
      break;
    }
    if (!doc.HasMember("lidar_configs") ||
        !doc["lidar_configs"].IsArray() ||
        0 == doc["lidar_configs"].Size()) {
      std::cout << "there is no user-defined config" << std::endl;
      break;
    }
    if (!ParseUserConfigs(doc, lidar_configs)) {
      std::cout << "failed to parse basic configs" << std::endl;
      break;
    }
    return true;
  } while (false);

  std::fclose(raw_file);
  return false;
}

bool LivoxLidarConfigParser::ParseUserConfigs(const rapidjson::Document &doc,
                                              std::vector<UserLivoxLidarConfig> &user_configs) {
  const rapidjson::Value &lidar_configs = doc["lidar_configs"];
  for (auto &config : lidar_configs.GetArray()) {
    if (!config.HasMember("ip")) {
      continue;
    }
    UserLivoxLidarConfig user_config;

    // parse user configs
    user_config.handle = IpStringToNum(std::string(config["ip"].GetString()));
    if (!config.HasMember("pcl_data_type")) {
      user_config.pcl_data_type = -1;
    } else {
      user_config.pcl_data_type = static_cast<int8_t>(config["pcl_data_type"].GetInt());
    }
    if (!config.HasMember("pattern_mode")) {
      user_config.pattern_mode = -1;
    } else {
      user_config.pattern_mode = static_cast<int8_t>(config["pattern_mode"].GetInt());
    }
    if (!config.HasMember("blind_spot_set")) {
      user_config.blind_spot_set = -1;
    } else {
      user_config.blind_spot_set = static_cast<int8_t>(config["blind_spot_set"].GetInt());
    }
    if (!config.HasMember("dual_emit_en")) {
      user_config.dual_emit_en = -1;
    } else {
      user_config.dual_emit_en = static_cast<uint8_t>(config["dual_emit_en"].GetInt());
    }
    if (!config.HasMember("extrinsic_parameter")) {
      memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param));
    } else {
      auto &value = config["extrinsic_parameter"];
      if (!ParseExtrinsics(value, user_config.extrinsic_param)) {
        memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param));
        std::cout << "failed to parse extrinsic parameters, ip: "
                  << IpNumToString(user_config.handle) << std::endl;
      }
    }
    if (!config.HasMember("angle_filter")) {
      user_config.enable_angle_filter = false;
      user_config.angle_filter_width = 0.0f;
      user_config.angle_filter_centers.clear();
    } else {
      auto &angle_filter = config["angle_filter"];
      if (angle_filter.HasMember("enable") && angle_filter["enable"].IsBool()) {
        user_config.enable_angle_filter = angle_filter["enable"].GetBool();
      } else {
        user_config.enable_angle_filter = true; // default true if member exists
      }

      if (angle_filter.HasMember("width") && angle_filter["width"].IsNumber()) {
        user_config.angle_filter_width = angle_filter["width"].GetFloat();
      } else {
        user_config.angle_filter_width = 0.0f;
      }

      if (angle_filter.HasMember("distance") && angle_filter["distance"].IsNumber()) {
        user_config.angle_filter_dist = angle_filter["distance"].GetFloat();
      } else {
        user_config.angle_filter_dist = 0.0f;
      }

      if (angle_filter.HasMember("centers") && angle_filter["centers"].IsArray()) {
        for (auto &center : angle_filter["centers"].GetArray()) {
          if (center.IsNumber()) {
            user_config.angle_filter_centers.push_back(center.GetFloat());
          }
        }
      }
    }
    user_config.set_bits = 0;
    user_config.get_bits = 0;

    user_configs.push_back(user_config);
  }

  if (0 == user_configs.size()) {
    std::cout << "no valid base configs" << std::endl;
    return false;
  }
  std::cout << "successfully parse base config, counts: "
            << user_configs.size() << std::endl;
  return true;
}

bool LivoxLidarConfigParser::ParseExtrinsics(const rapidjson::Value &value,
                                             ExtParameter &param) {
  param.roll = GetJsonFloat(value, "roll", 0.0f);
  param.pitch = GetJsonFloat(value, "pitch", 0.0f);
  param.yaw = GetJsonFloat(value, "yaw", 0.0f);
  param.x = GetJsonInt32(value, "x", 0);
  param.y = GetJsonInt32(value, "y", 0);
  param.z = GetJsonInt32(value, "z", 0);

  return true;
}

} // namespace livox_ros
