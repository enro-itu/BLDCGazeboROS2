from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('bldc_gz_sim')
    world_launch = os.path.join(pkg, 'launch', 'bldc_world.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(world_launch)),
        Node(package='bldc_gz_sim', executable='gui.py', name='bldc_motor_gui', output='screen'),
    ])
