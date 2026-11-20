#!/usr/bin/env python3
"""Ad-hoc position-hold test: locks the Falcon at whatever position it's at
when this starts, using a spring-damper force. Ctrl+C to release."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3


class PositionHold(Node):
    def __init__(self):
        super().__init__('position_hold')
        self.target = None
        self.last_pos = None
        self.last_time = None
        self.velocity = (0.0, 0.0, 0.0)
        self.k = 150.0   # N/m, spring stiffness
        self.d = 12.0    # N/(m/s), damping
        self.max_f = 4.0  # N, stay under the node's 5N clamp

        self.pos_sub = self.create_subscription(
            PointStamped, 'falcon/position', self.on_position, 10)
        self.force_pub = self.create_publisher(Vector3, 'falcon/force', 10)
        self.timer = self.create_timer(0.01, self.on_timer)  # 100 Hz

    def on_position(self, msg: PointStamped):
        now = self.get_clock().now()
        pos = (msg.point.x, msg.point.y, msg.point.z)
        if self.target is None:
            self.target = pos
            self.get_logger().info(f'Locking target at {pos}')
        if self.last_pos is not None and self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 0:
                self.velocity = tuple(
                    (pos[i] - self.last_pos[i]) / dt for i in range(3))
        self.last_pos = pos
        self.last_time = now

    def on_timer(self):
        if self.target is None or self.last_pos is None:
            return
        ex = self.target[0] - self.last_pos[0]
        ey = self.target[1] - self.last_pos[1]
        ez = self.target[2] - self.last_pos[2]

        vx, vy, vz = self.velocity
        fx = self.k * ex - self.d * vx
        fy = self.k * ey - self.d * vy
        fz = self.k * ez - self.d * vz

        mag = (fx**2 + fy**2 + fz**2) ** 0.5
        if mag > self.max_f:
            scale = self.max_f / mag
            fx *= scale
            fy *= scale
            fz *= scale

        msg = Vector3()
        msg.x = fx
        msg.y = fy
        msg.z = fz
        self.force_pub.publish(msg)


def main():
    rclpy.init()
    node = PositionHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero = Vector3()
        node.force_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
