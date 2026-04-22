import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cur_path = os.path.split(os.path.realpath(__file__))[0]
    config_path = os.path.join(cur_path, "../config/multi_MID360_config.json")

    calibrator = Node(
        package="livox_ros_driver2",
        executable="multi_lidar_map_calibrator_node",
        name="multi_lidar_map_calibrator",
        output="screen",
        parameters=[{
            "config_path": config_path,
            "map_pcd_path": "/home/chen/Workspace/algor_ws/src/faster-slam/prior/company/PGO.pcd",
            "output_path": "/tmp/multi_lidar_map_calibration.yaml",
            "debug_pcd_dir": "/tmp/multi_lidar_map_calibrator_debug",
            "save_debug_pcd": True,
            "lidar_topics": [
                "/livox/lidar_192_168_2_202",
                "/livox/lidar_192_168_2_203",
                "/livox/lidar_192_168_2_204",
                "/livox/lidar_192_168_2_205",
            ],
            # Initial map->base pose: [x, y, z, roll, pitch, yaw], in meters / radians.
            # The node will combine this pose with each lidar's base->lidar extrinsic
            # from multi_MID360_config.json to form the initial map->lidar guesses.
            # Converted from the provided quaternion:
            # position = [1.8243002867857288, 3.126755202549615, 0.12541906790287197]
            # orientation(xyzw) = [0.007924878208096809, 0.0007758747570828233,
            #                      0.6891718699743805, 0.7245541896783062]
            "initial_base_pose": [
                1.8243002867857288,
                3.126755202549615,
                0.12541906790287197,
                0.012554362036105247,
                -0.009799036474483087,
                1.5206898970322682,
            ],
            # When the robot stays still, accumulating multiple frames usually makes
            # map registration noticeably more stable than using a single scan.
            "required_accumulated_frames": 10,
            "status_log_interval_frames": 5,
            "source_voxel_size": 0.05,
            "target_voxel_size": 0.05,
            "registration_source_voxel_size": 0.05,
            "registration_target_voxel_size": 0.05,
            "crop_half_extent_xy": 6.0,
            "crop_half_extent_z": 2.5,
            "max_correspondence_distance": 1.0,
            "transformation_epsilon": 1e-4,
            "fitness_epsilon": 1e-3,
            "maximum_iterations": 80,
            "registration_threads": 4,
            "run_once": True,
        }]
    )

    return LaunchDescription([calibrator])
