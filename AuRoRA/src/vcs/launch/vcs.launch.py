from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    role = LaunchConfiguration('role')

    return LaunchDescription([

        DeclareLaunchArgument(
            'role',
            default_value='robot',
            description='robot or server'
        ),

        Node(
            package='scs',
            executable='scs_igniter',
            name='scs_igniter',
            output='screen',
        ),

        Node(
            package='vcs',
            executable='vital_terminal_core',
            name='vital_terminal_core',
            output='screen',
            parameters=[{'role': role}],
            condition=IfCondition(
                PythonExpression(["'", role, "' == 'robot'"])
            )
        ),

        Node(
            package='vcs',
            executable='vital_central_core',
            name='vital_central_core',
            output='screen',
            parameters=[{'role': role}],
            condition=IfCondition(
                PythonExpression(["'", role, "' == 'server'"])
            )
        ),
    ])
