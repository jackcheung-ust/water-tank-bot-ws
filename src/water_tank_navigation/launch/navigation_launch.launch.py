import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_dir = LaunchConfiguration('map', default= os.path.join(get_package_share_directory('water_tank_navigation'),'map','testing_map.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # rviz2_config = os.path.join(get_package_share_directory('water_tank_navigation'), 'config','config2.rviz')

    parm_file = "turtlebot3.yaml"

    parm_dir = LaunchConfiguration('parm_file', default= os.path.join(get_package_share_directory('water_tank_navigation'),'config',parm_file))

    map_arg = DeclareLaunchArgument('map', default_value=map_dir, description='Full path to map file to load')

    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    parm_arg = DeclareLaunchArgument('parm', default_value=parm_dir, description='Full path to param file to load')

    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': parm_dir}.items(),
        )
    
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=[
    #         "-d",
    #         rviz2_config,
    #     ],
    #     output="screen",
    # )

    

    return LaunchDescription([
        map_arg,
        sim_time_arg,
        parm_arg,
        nav2,
        
        # rviz,
    ])