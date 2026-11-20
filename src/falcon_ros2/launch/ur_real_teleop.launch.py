"""Falcon -> real UR3e teleop.

Brings up:
  - ur_robot_driver for the real robot (+ RViz)
  - MoveIt move_group (for /compute_ik)
  - falcon_node (skip with use_falcon:=false, e.g. to run it in its own terminal)
  - falcon_ur_teleop.py

Unlike the sim launch files this does NOT run move_to_ready.py: on the real arm, jog it to an
open pose with the elbow bent from the pendant first. Then press Play on the External Control
program and wait for "Robot connected to reverse interface".

  ros2 launch falcon_ros2 ur_real_teleop.launch.py robot_ip:=<ROBOT_IP> \
      [kinematics_params_file:=$HOME/ur3e_calibration.yaml]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    use_falcon = LaunchConfiguration('use_falcon')

    ur_control = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py']),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': LaunchConfiguration('robot_ip'),
            'kinematics_params_file': LaunchConfiguration('kinematics_params_file'),
            'launch_rviz': 'true',
        }.items())

    moveit = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']),
        launch_arguments={'ur_type': ur_type, 'launch_rviz': 'false'}.items())

    falcon = Node(
        package='falcon_ros2', executable='falcon_node', output='screen',
        # The haptic loop stays ~1 kHz inside falcon_node; 250 Hz is plenty for the bridge
        parameters=[{'publish_rate': 250.0}],
        condition=IfCondition(use_falcon))

    # Run through python3 so it works even when the source lacks +x (see the sim launch files)
    teleop = ExecuteProcess(
        cmd=['python3', PathJoinSubstitution(
            [FindPackagePrefix('falcon_ros2'), 'lib', 'falcon_ros2', 'falcon_ur_teleop.py']),
             '--ros-args', '-p', ['mode:=', LaunchConfiguration('mode')],
             '-p', ['log_file:=', LaunchConfiguration('log_file')],
             '-p', ['force_feedback:=', LaunchConfiguration('force_feedback')]],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', description='IP address of the UR3e'),
        DeclareLaunchArgument('ur_type', default_value='ur3e'),
        DeclareLaunchArgument('use_falcon', default_value='true'),
        DeclareLaunchArgument('mode', default_value='rate', description="Teleop control: 'rate' (joystick), 'position', or 'twist' (joystick via twist_controller + Insert force mode, real robot only)"),
        DeclareLaunchArgument('force_feedback', default_value='true',
                              description='Contact force feedback on the Falcon (false: only the joystick spring)'),
        DeclareLaunchArgument('log_file', default_value="''",
                              description='CSV file or directory to log force feedback to (empty: off)'),
        DeclareLaunchArgument(
            'kinematics_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ur_description'), 'config', ur_type, 'default_kinematics.yaml']),
            description='Calibration file from ur_calibration (recommended)'),
        ur_control,
        moveit,
        falcon,
        teleop,
    ])
