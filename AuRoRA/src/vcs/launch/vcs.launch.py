from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    """
    Create a LaunchDescription that declares a 'role' launch argument and configures nodes based on its value.
    
    The launch description declares the 'role' argument (default 'robot'), always launches the 'scs' package's 'igniter' node, and conditionally launches one of two 'vcs' package nodes:
    - 'vital_terminal_core' when role == 'robot'
    - 'vital_central_core' when role == 'server'
    
    Returns:
        LaunchDescription: The constructed launch description containing the argument and node actions.
    """
    role = LaunchConfiguration('role')

    return LaunchDescription([

        DeclareLaunchArgument(
            'role',
            default_value='robot',
            description='robot or server'
        ),

        Node(
            package='scs',
            executable='igniter',
            name='igniter',
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