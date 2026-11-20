"""Falcon -> simulated UR (mock hardware) teleop, visualized in RViz.

Brings up:
  - ur_robot_driver with mock hardware (no robot/URSim needed)
  - MoveIt move_group (for /compute_ik) + RViz
  - falcon_node (skip with use_falcon:=false, e.g. to run it in its own terminal)
  - move_to_ready.py, then falcon_ur_teleop.py once the arm is out of its singular start pose
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    use_falcon = LaunchConfiguration('use_falcon')

    # Scripts run through python3 so they work even when the source lacks +x
    # (e.g. on a VMware/VirtualBox shared folder).
    def script(name):
        return PathJoinSubstitution([FindPackagePrefix('falcon_ros2'), 'lib', 'falcon_ros2', name])

    ur_control = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py']),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': '0.0.0.0',
            'use_mock_hardware': 'true',
            'launch_rviz': 'false',
        }.items())

    moveit = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']),
        launch_arguments={'ur_type': ur_type, 'launch_rviz': 'true'}.items())

    falcon = Node(
        package='falcon_ros2', executable='falcon_node', output='screen',
        # The haptic loop stays ~1 kHz inside falcon_node; 250 Hz is plenty for the bridge
        parameters=[{'publish_rate': 250.0}],
        condition=IfCondition(use_falcon))

    move_to_ready = ExecuteProcess(cmd=['python3', script('move_to_ready.py')], output='screen')
    teleop = ExecuteProcess(
        cmd=['python3', script('falcon_ur_teleop.py'),
             '--ros-args', '-p', ['mode:=', LaunchConfiguration('mode')],
             '-p', ['log_file:=', LaunchConfiguration('log_file')]],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3e'),
        DeclareLaunchArgument('use_falcon', default_value='true'),
        DeclareLaunchArgument('mode', default_value='rate', description="Teleop control: 'rate' (joystick), 'position', or 'twist' (joystick via twist_controller + Insert force mode, real robot only)"),
        DeclareLaunchArgument('log_file', default_value="''",
                              description='CSV file or directory to log force feedback to (empty: off)'),
        ur_control,
        moveit,
        falcon,
        move_to_ready,
        RegisterEventHandler(OnProcessExit(target_action=move_to_ready, on_exit=[teleop])),
    ])
