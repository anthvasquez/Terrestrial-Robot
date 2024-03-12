import os
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    share_dir = get_package_share_directory('Terrestrial_Robot_Export_description')
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare('terrestrial_robot'),
            'bringup',
            'controller.yaml'
        ]
    )

    xacro_file = os.path.join(share_dir, 'urdf', 'Terrestrial_Robot_Export.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Launch Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Start the controller manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",
    )

    # Publish the urdf on 'robot_description' topic and repost positions to /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawn the robot defined on 'robot_description'
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'Terrestrial_Robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Load the controllers
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "velocity_controller",
            ]
    )


    # Ensure nodes launch in the correct order

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        #gazebo_node,
        #urdf_spawn_node,
        controller_spawner,
        #joint_state_broadcaster_spawner
    ])