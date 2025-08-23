from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ],
            output='screen'
        )
    ])
