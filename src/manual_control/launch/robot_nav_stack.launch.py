import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('manual_control')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    params_file = LaunchConfiguration('params_file')
    camera_model = LaunchConfiguration('camera_model')
    serial_port = LaunchConfiguration('serial_port')
    launch_zed = LaunchConfiguration('launch_zed')
    launch_rtabmap = LaunchConfiguration('launch_rtabmap')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_bridge = LaunchConfiguration('launch_bridge')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')
    rtabmap_database_path = LaunchConfiguration('rtabmap_database_path')
    rtabmap_args = LaunchConfiguration('rtabmap_args')

    default_params = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    zed_params_override = os.path.join(package_dir, 'config', 'zed_nav_override.yaml')

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        condition=IfCondition(launch_zed),
        launch_arguments={
            'camera_name': 'zed',
            'camera_model': camera_model,
            'ros_params_override_path': zed_params_override,
            'publish_tf': 'true',
            # Let RTAB-Map own map->odom to avoid conflicting map sources.
            'publish_map_tf': 'false',
            'publish_urdf': 'true',
            'publish_imu_tf': 'false',
            'node_log_type': 'screen',
        }.items(),
    )

    base_link_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_base_link_static_tf',
        condition=IfCondition(launch_zed),
        output='screen',
        arguments=[
            '--x', '-0.2',
            '--y', '0.0',
            '--z', '-0.15',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'zed_camera_link',
            '--child-frame-id', 'base_link',
        ],
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        condition=IfCondition(launch_rtabmap),
        launch_arguments={
            'namespace': 'rtabmap',
            'rtabmap_viz': 'false',
            'rviz': use_rviz,
            'database_path': rtabmap_database_path,
            'args': rtabmap_args,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_topic': '/zed/zed_node/odom',
            'rgb_topic': '/zed/zed_node/rgb/color/rect/image',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
            'visual_odometry': 'false',
            'publish_tf_map': 'true',
            'approx_sync': 'true',
            'wait_for_transform': '0.5',
            'topic_queue_size': '30',
            'queue_size': '30',
            'qos': '1',
            'log_level': log_level,
            'output_goal_topic': '/goal_pose',
            'use_action_for_goal': 'false',
        }.items(),
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        condition=IfCondition(launch_nav2),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        condition=IfCondition(launch_nav2),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        condition=IfCondition(launch_nav2),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        condition=IfCondition(launch_nav2),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        condition=IfCondition(launch_nav2),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        condition=IfCondition(launch_nav2),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        condition=IfCondition(launch_nav2),
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        condition=IfCondition(launch_nav2),
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    cmd_vel_bridge = Node(
        package='manual_control',
        executable='cmd_vel_serial_bridge',
        name='cmd_vel_serial_bridge',
        condition=IfCondition(launch_bridge),
        output='screen',
        parameters=[{
            'topic_name': '/cmd_vel',
            'serial_port': serial_port,
            'baud_rate': 115200,
            'frame_id': 'CMD_VEL',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 params file.',
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zedm',
            description='ZED camera model. Override if your hardware differs.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial device for the cmd_vel bridge.',
        ),
        DeclareLaunchArgument(
            'launch_zed',
            default_value='true',
            description='Launch the ZED camera wrapper.',
        ),
        DeclareLaunchArgument(
            'launch_rtabmap',
            default_value='true',
            description='Launch RTAB-Map SLAM/localization.',
        ),
        DeclareLaunchArgument(
            'launch_nav2',
            default_value='true',
            description='Launch the Nav2 navigation stack.',
        ),
        DeclareLaunchArgument(
            'launch_bridge',
            default_value='true',
            description='Launch the serial cmd_vel bridge.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz from RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level.',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path',
            default_value='~/.ros/rtabmap.db',
            description='RTAB-Map database path used to load/save the map.',
        ),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='',
            description='Extra RTAB-Map CLI args, e.g. --delete_db_on_start.',
        ),
        zed_launch,
        base_link_static_tf,
        rtabmap_launch,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        cmd_vel_bridge,
    ])
