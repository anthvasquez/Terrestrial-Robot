import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    launchArgument = DeclareLaunchArgument("simulation", default_value="false", description="Launch robot in sim mode")

    description_dir = get_package_share_directory('terrestrial_robot')
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare('terrestrial_robot'),
            'config',
            'wheel_test.yaml'
        ]
    )

    xacro_file = os.path.join(description_dir, 'description', 'urdf', 'terrestrial_robot.xacro')
    robot_urdf = xacro.process_file(xacro_file, mappings={"simulation" : 'false'}).toxml()

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
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
        output="both"
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
        launchArgument,
        
        control_node,
        robot_state_publisher_node,
        #gazebo_node,
        #urdf_spawn_node,
        controller_spawner
    ])