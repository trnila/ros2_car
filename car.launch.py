from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(package='move_base_ackermann', executable='node'),
        Node(executable='ultrasonic/ultrasonic'),
        Node(executable='../+_repo_rules+camera_ros/camera_ros'),
    ])
