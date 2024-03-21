import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    motor_config = "quadmotor"

    include_motors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('motorspeedcontroller'),
                'launch',
                f'{motor_config}.py'))
    )

    return LaunchDescription([
        include_motors
    ])