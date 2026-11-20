"""Falcon -> UR in Gazebo, with force feedback when the arm hits an obstacle.

Brings up:
  - ur_simulation_gz (Gazebo + gz_ros2_control) with worlds/obstacles.sdf
  - MoveIt move_group (for /compute_ik) + RViz, on sim time
  - a ros_gz bridge for the obstacles' contact sensors (/obstacle_contacts)
  - falcon_node (skip with use_falcon:=false, e.g. to run it in its own terminal)
  - move_to_ready.py, then falcon_ur_teleop.py once the arm is out of its singular start pose

Needs: sudo apt install ros-jazzy-ur-simulation-gz  (pulls in gz_ros2_control)
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
        return ['python3',
                PathJoinSubstitution([FindPackagePrefix('falcon_ros2'), 'lib', 'falcon_ros2', name]),
                '--ros-args', '-p', 'use_sim_time:=true']

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ur_simulation_gz'), 'launch', 'ur_sim_control.launch.py']),
        launch_arguments={
            'ur_type': ur_type,
            'world_file': LaunchConfiguration('world_file'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': 'false',
        }.items())

    moveit = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']),
        launch_arguments={'ur_type': ur_type, 'use_sim_time': 'true', 'launch_rviz': 'true'}.items())

    contacts_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/obstacle_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'])

    # falcon_node talks to real hardware, so it stays on wall time
    falcon = Node(
        package='falcon_ros2', executable='falcon_node', output='screen',
        # The haptic loop stays ~1 kHz inside falcon_node; 250 Hz is plenty for the bridge
        parameters=[{'publish_rate': 250.0}],
        condition=IfCondition(use_falcon))

    move_to_ready = ExecuteProcess(cmd=script('move_to_ready.py'), output='screen')
    teleop = ExecuteProcess(
        cmd=script('falcon_ur_teleop.py') + ['-p', ['mode:=', LaunchConfiguration('mode')],
             '-p', ['log_file:=', LaunchConfiguration('log_file')]],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur3e'),
        DeclareLaunchArgument('use_falcon', default_value='true'),
        DeclareLaunchArgument('mode', default_value='rate', description="Teleop control: 'rate' (joystick), 'position', or 'twist' (joystick via twist_controller + Insert force mode, real robot only)"),
        DeclareLaunchArgument('log_file', default_value="''",
                              description='CSV file or directory to log force feedback to (empty: off)'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'world_file',
            default_value=PathJoinSubstitution([FindPackageShare('falcon_ros2'), 'worlds', 'obstacles.sdf'])),
        gazebo,
        moveit,
        contacts_bridge,
        falcon,
        move_to_ready,
        RegisterEventHandler(OnProcessExit(target_action=move_to_ready, on_exit=[teleop])),
    ])
