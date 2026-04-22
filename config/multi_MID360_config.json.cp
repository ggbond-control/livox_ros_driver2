{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "host_ip"        : "192.168.2.99",
      "multicast_ip"   : "224.1.1.5",
      "cmd_data_port"  : 56101,
      "push_msg_port"  : 56201,
      "point_data_port": 56301,
      "imu_data_port"  : 56401,
      "log_data_port"  : 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.2.202",         
      "pcl_data_type" : 1,
      "pattern_mode" : 0,         
      "extrinsic_parameter" : {
        "roll": 95.0,
        "pitch": 0.0,
        "yaw": -90.0,
        "x": -465.76,
        "y": 0.0,
        "z": 34.84
      }
    },
    {
      "ip" : "192.168.2.203",    
      "pcl_data_type" : 1,
      "pattern_mode" : 0,         
      "extrinsic_parameter" : {
        "roll": 146.0,
        "pitch": 0.0,
        "yaw": -90.0,
        "x": -410.74,
        "y": 0.0,
        "z": -56.01
      }
    },
    {
      "ip" : "192.168.2.204",     
      "pcl_data_type" : 1,
      "pattern_mode" : 0,         
      "extrinsic_parameter" : {
        "roll": 136.0,
        "pitch": 0.0,
        "yaw": 90.0,
        "x": 432.97,
        "y": 0.0,
        "z": -55.19
      }
    },
    {
      "ip" : "192.168.2.205",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 65.0,
        "pitch": 0.0,
        "yaw": 90.0,
        "x": 443.67,
        "y": 0.0,
        "z": 54.15
      }
    }
  ]
}