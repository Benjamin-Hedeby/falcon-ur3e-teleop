"""Falcon -> falcon_ur_teleop.py (twist mode) -> live MuJoCo UR3e with the peg and socket.

Brings up:
  - the MuJoCo peg-in-hole simulation as a stand-in UR3e + driver (mujoco/peg_in_hole/sim_ros.py,
    run with the MuJoCo venv), with its viewer window; closing the window stops everything
  - falcon_node (skip with use_falcon:=false)
  - falcon_ur_teleop.py in twist mode: the same bridge and admittance code as the real robot

Drive with the Falcon: hold a button (not minus) as the clutch, tap minus for the admittance
Insert. Reset the arm with: ros2 service call /mujoco/reset std_srvs/srv/Trigger

Needs the MuJoCo setup in mujoco_dir (default ~/ur3e_ws/mujoco: .venv + peg_in_hole/meshes).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    mujoco_dir = LaunchConfiguration('mujoco_dir')
    sim_dir = PathJoinSubstitution([mujoco_dir, 'peg_in_hole'])

    sim_cmd = [PathJoinSubstitution([mujoco_dir, '.venv', 'bin', 'python']),
               PathJoinSubstitution([sim_dir, 'sim_ros.py']),
               '--offset', LaunchConfiguration('offset_x'), LaunchConfiguration('offset_y')]
    viewer = LaunchConfiguration('viewer')
    sim = ExecuteProcess(cmd=sim_cmd, cwd=sim_dir, output='screen', condition=IfCondition(viewer))
    sim_headless = ExecuteProcess(cmd=sim_cmd + ['--headless'], cwd=sim_dir, output='screen',
                                  condition=UnlessCondition(viewer))

    falcon = Node(
        package='falcon_ros2', executable='falcon_node', output='screen',
        parameters=[{'publish_rate': 250.0}],
        condition=IfCondition(LaunchConfiguration('use_falcon')))

    teleop = ExecuteProcess(
        cmd=['python3', PathJoinSubstitution(
            [FindPackagePrefix('falcon_ros2'), 'lib', 'falcon_ros2', 'falcon_ur_teleop.py']),
             '--ros-args', '-p', 'mode:=twist',
             '-p', ['log_file:=', LaunchConfiguration('log_file')],
             '-p', ['force_feedback:=', LaunchConfiguration('force_feedback')]],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('mujoco_dir', default_value=os.path.expanduser('~/ur3e_ws/mujoco')),
        DeclareLaunchArgument('offset_x', default_value='0.010',
                              description='Hole offset from the start position along X, m'),
        DeclareLaunchArgument('offset_y', default_value='0.006',
                              description='Hole offset from the start position along Y, m'),
        DeclareLaunchArgument('use_falcon', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true',
                              description='MuJoCo viewer window (false: headless, for tests)'),
        DeclareLaunchArgument('force_feedback', default_value='true',
                              description='Contact force feedback on the Falcon (false: only the joystick spring)'),
        DeclareLaunchArgument('log_file', default_value="''",
                              description='CSV file or directory to log force feedback to (empty: off)'),
        sim,
        sim_headless,
        falcon,
        teleop,
        # Closing the viewer window ends the whole session
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown())])),
        RegisterEventHandler(OnProcessExit(target_action=sim_headless,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
