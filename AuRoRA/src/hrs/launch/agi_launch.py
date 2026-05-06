# launch/agi_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hrs",
            executable="ras",
            name="ras"
        ),
        Node(
            package="scs",
            executable="cnc",
            name="cnc"
        ),
    ])
