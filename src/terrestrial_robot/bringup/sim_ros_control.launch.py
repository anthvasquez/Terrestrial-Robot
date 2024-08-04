import os
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    simulation = LaunchConfiguration('simulation')  #This will have the value of the launch argument once it gets the launch context (it's not at that point yet)
    sim_launchArgument = DeclareLaunchArgument("simulation", default_value="true", description="Launch robot in sim mode")  #allow the command line to read in argument
    robot_dir = get_package_share_directory('terrestrial_robot')

    # To pass launch arguments into xacro file, we must use ros Command with substitutions to fill in the argument once it gets the launch context
    xacro_file = os.path.join(robot_dir, 'description', 'urdf', 'terrestrial_robot.xacro')
    #robot_urdf = xacro.process_file(xacro_file, mappings={'simulation': 'true'}).toxml()

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

    # Publish the urdf on 'robot_description' topic and repost positions to /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro ', xacro_file, ' simulation:=', simulation])}
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
    # sim_controller.yaml is loaded from he <gazebo> tag in the xacro
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "mecanum_controller",
            ]
    )


    # Ensure nodes launch in the correct order

    return LaunchDescription([
        sim_launchArgument,
        
        robot_state_publisher_node,
        gazebo_node,
        urdf_spawn_node,
        controller_spawner,
    ])