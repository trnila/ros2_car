from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='move_base_ackermann', executable='node'),
        Node(package='ultrasonic', executable='node'),
        Node(package='v4l2_camera', executable='v4l2_camera_node'),
    ])
