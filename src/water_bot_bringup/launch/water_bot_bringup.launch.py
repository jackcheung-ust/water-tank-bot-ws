#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    rf2o_dir = get_package_share_directory('rf2o_laser_odometry')
    water_bot_control_dir = get_package_share_directory('water_bot_control')

    # 1. Launch RPLidar C1 first
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(rplidar_ros_dir, 'launch', 'rplidar_c1_launch.py')
        ])
    )

    # 2. Launch rf2o laser odometry after 2 seconds (wait for RPLidar to be ready)
    rf2o_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(rf2o_dir, 'launch', 'rf2o_laser_odometry.launch.py')
                ])
            )
        ]
    )

    # 3. Launch water bot control (can start in parallel)
    water_bot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(water_bot_control_dir,'launch', 'diffbot.launch.py')
        ])
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch actions
    ld.add_action(rplidar_launch)
    ld.add_action(rf2o_launch)
    ld.add_action(water_bot_control_launch)

    return ld
