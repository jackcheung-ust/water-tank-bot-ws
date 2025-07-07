import os 
import xacro 
from ament_index_python.packages import get_package_share_path
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    water_tank_carto_path = get_package_share_path('water_tank_cartographer')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    lua_configuration = os.path.join(water_tank_carto_path,'config')
    lua_file_name = 'cart_slam_2d.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time},],
        arguments=['-configuration_directory', lua_configuration, '-configuration_basename', lua_file_name],
        # remappings=[('odom', '/odom_rf2o')],
        output='screen',
        )
    
    cartographer_map = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05','-publish_period_sec','1.0'],
        # remappings=[('odom', '/odom_rf2o')],
        output='screen',
    )

    return LaunchDescription([
        cartographer_node,
        cartographer_map,
        ])