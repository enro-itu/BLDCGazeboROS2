from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bldc_gz_sim')
    world = os.path.join(pkg_share, 'worlds', 'bldc_world.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', 'world'],
            output='screen'
        )
    ])