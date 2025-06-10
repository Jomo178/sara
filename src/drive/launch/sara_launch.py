import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr.yaml'
    )

    # Declare map argument with default path to your map
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join('/home/sara/sara', 'raum.yaml'),
        description='Full path to map yaml file to load'
    )

    # Static transform from base_link to ldlidar_base
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_ldlidar_tf',
        arguments=['0.17', '0', '0.13', '0', '0', '0', 'base_link', 'ldlidar_base']
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[ 
            lc_mgr_config_path  # Parameters
        ]
    )

    # Include LDLidar launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': 'ldlidar_node'
        }.items()
    )

    #rf20 laser_odometry launch
    rf20_laser_launch_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/ldlidar_node/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }],
    )

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_params_file = os.path.join(
        get_package_share_directory('drive'),
        'params',
        'nav2_params.yaml'
    )

    bringup_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file,
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'False'
        }.items()
    )

    #launch rviz2
    rviz_config_file = os.path.join(
        get_package_share_directory('drive'),
        'params',
        'rviz2.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add map argument declaration
    ld.add_action(declare_map_yaml_cmd)

    # static transform publisher from base_link to ldlidar_base
    ld.add_action(base_to_lidar_tf)

    # Launch Nav2 Lifecycle Manager
    ld.add_action(lc_mgr_node)

    # Call LDLidar launch
    ld.add_action(ldlidar_launch)

    # Call RF2O Laser Odometry launch
    ld.add_action(rf20_laser_launch_odometry)

    # Add Nav2 bringup launch
    ld.add_action(bringup_launch)

    # Add RViz2 node
    ld.add_action(rviz_node)

    return ld

