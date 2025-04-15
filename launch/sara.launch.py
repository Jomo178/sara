import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    node_name = LaunchConfiguration('node_name')

    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('drive'),
        'params',
        'lifecycle_mgr_slam.yaml'
    )

        # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(
        get_package_share_directory('drive'),
        'params',
        'mapper_params_online_async.yaml'
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
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

    # Fake odom publisher
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'ldlidar_base']
    )

    scan_listener = Node(
        package="drive",
        executable="scan_listener",
        name="scan_listener",
        output="screen"
    )


    # SLAM Toolbox node in async mode
    slam_toolbox_node = LifecycleNode(
          package='slam_toolbox',
          executable='async_slam_toolbox_node',
          namespace='',
          name='slam_toolbox',
          output='screen',
          parameters=[
            # YAML files
            slam_config_path, # Parameters
            {'use_sim_time': False}, 
            {'scan_topic': '/ldlidar_node/scan'}, # Topic remapping 
          ],
          remappings=[
              ('/scan', '/ldlidar_node/scan')
          ]          
    )

       # RVIZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'config',
        'ldlidar_slam.rviz'
    )

    # RVIZ2node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )

    ld = LaunchDescription()

    # Launch Nav2 Lifecycle Manager
    ld.add_action(lc_mgr_node)

        # Launch SLAM Toolbox node
    ld.add_action(slam_toolbox_node)
    # Launch fake odom publisher node
    ld.add_action(fake_odom)
    # Call LDLidar launch
    ld.add_action(ldlidar_launch)
    # Launch scan listener node
    #ld.add_action(scan_listener)

        # Start RVIZ2
    ld.add_action(rviz2_node)


    return ld

#ros2 launch slam_toolbox online_async_launch.py params_file:=/src/drive/params/mapper_params_online_async.yaml use_sim_time:=false scan_topic:=/ldlidar_node/scan