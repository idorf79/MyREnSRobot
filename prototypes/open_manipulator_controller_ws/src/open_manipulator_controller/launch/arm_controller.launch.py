"""
Launch the OpenManipulatorX arm controller (MoveIt backend).

This launch file starts only the arm_controller node itself.
MoveIt's move_group must already be running before this node starts.

Real robot:
  1. ros2 launch open_manipulator_x_moveit_config move_group.launch.py
  2. ros2 launch open_manipulator_controller arm_controller.launch.py

Gazebo simulation:
  1. ros2 launch open_manipulator_x_moveit_config move_group.launch.py use_sim_time:=true
  2. ros2 launch open_manipulator_controller arm_controller.launch.py use_sim_time:=true

RViz mock joints (no hardware, no Gazebo):
  1. ros2 launch open_manipulator_x_moveit_config move_group.launch.py
  2. ros2 launch open_manipulator_controller arm_controller.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('open_manipulator_controller')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg, 'config', 'controller_params.yaml']),
        description='Path to ROS2 parameter file',
    )

    arm_controller_node = Node(
        package='open_manipulator_controller',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        arm_controller_node,
    ])
