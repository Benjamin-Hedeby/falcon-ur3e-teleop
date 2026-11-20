#!/usr/bin/env python3
"""Move the UR arm to a bent, non-singular 'ready' pose, then exit.

UR mock hardware starts with the arm pointing straight up (elbow_joint = 0),
which is a kinematic singularity: /compute_ik fails or jumps wildly from
there. Running this once after startup gives the teleop bridge a sane start.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_ORDER = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
READY = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # tool pointing down, elbow bent


def main():
    rclpy.init()
    node = Node('move_to_ready')
    client = ActionClient(
        node, FollowJointTrajectory,
        '/scaled_joint_trajectory_controller/follow_joint_trajectory')
    node.get_logger().info('Waiting for scaled_joint_trajectory_controller...')
    client.wait_for_server()

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_ORDER
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=READY, time_from_start=Duration(sec=3))]
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        node.get_logger().error('Ready-pose goal rejected.')
    else:
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        node.get_logger().info('Arm at ready pose.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
