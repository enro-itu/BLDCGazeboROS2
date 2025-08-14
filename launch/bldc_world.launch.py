from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os, sys

def _check_and_launch(context, *args, **kwargs):
    pkg_share = get_package_share_directory('bldc_gz_sim')
    world_default = os.path.join(pkg_share, 'worlds', 'bldc_world.sdf')
    world = LaunchConfiguration('world').perform(context) or world_default

    if not os.path.isabs(world):
        world = os.path.join(pkg_share, world)

    if not os.path.exists(world):
        print(f"[bldc_gz_sim] World not found: {world}", file=sys.stderr)
        print(f"[bldc_gz_sim] pkg_share is: {pkg_share}", file=sys.stderr)
        raise RuntimeError("World file missing. Did you install `worlds/` and rebuild?")

    return [ExecuteProcess(cmd=['gz', 'sim', '-v', '4', world], output='screen')]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Absolute path or relative path under package share to the SDF world'
        ),
        OpaqueFunction(function=_check_and_launch)
    ])
