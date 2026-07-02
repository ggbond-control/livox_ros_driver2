import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
merge_lidars  = 1    # Keep merged publish enabled by default for the multi-MID360 launch.
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
################### user configure parameters for ros2 end #####################


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    config_file = {
        'mid360': 'multi_MID360_config.json',
        'mid360s': 'multi_MID360s_config.json',
    }[model]
    user_config_path = os.path.join(cur_config_path, config_file)

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"merge_lidars": merge_lidars},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    return [livox_driver]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='mid360',
            choices=['mid360', 'mid360s'],
            description='LiDAR config profile: mid360 or mid360s.'
        ),
        OpaqueFunction(function=launch_setup),
    ])
