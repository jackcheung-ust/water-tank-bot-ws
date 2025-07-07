import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import NotEqualsSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description()-> LaunchDescription:

    namespace = LaunchConfiguration('namespace')
    config_dir = get_package_share_directory('water_tank_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'amcl',
        'bt_navigator',
        'behavior_server',
        'controller_server',
        'map_server',
        'smoother_server',
        'planner_server',
        'velocity_smoother',
        'collision_monitor',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation (Gazebo) clock if true'
    )

    declare_parms_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir,'config', 'test_config.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True', description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composable nodes if true, otherwise use regular nodes',
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(config_dir, 'map', 'testing_map.yaml'),
        description='Full path to map file to load',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='Name of the container for composable nodes',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions= [
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package = 'nav2_amcl', 
                executable= 'amcl',
                name = 'amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package = 'nav2_behaviors',
                executable = 'behavior_server',
                name = 'behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),

            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),

            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings+ [('cmd_vel', 'cmd_vel_nav')],
            ),

            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        name='amcl',
                        parameters=[configured_params],
                        remappings=remappings,
                        # extra_arguments=[{'use_intra_process_comms': True}],
                    ),

                    ComposableNode(
                        package = 'nav2_bt_navigator',
                        plugin = 'nav2_bt_navigator::BtNavigator',
                        name = 'bt_navigator',
                        parameters = [configured_params],
                        remappings = remappings,
                    ),

                    ComposableNode(
                        condition=IfCondition(
                            NotEqualsSubstitution(LaunchConfiguration('map_file'), '')
                        ),
                        package = 'nav2_map_server',
                        plugin = 'nav2_map_server::MapServer',
                        name = 'map_server',
                        parameters = [configured_params, {'yaml_filename': map_file}],
                        remappings = remappings,
                    ),

                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),

                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),

                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='nav2_behaviors::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),



                ],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_parms_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(load_nodes)


    return ld 
