import math
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, JointState
from sensor_msgs_py import point_cloud2


class ReconstructionNode(Node):
    def __init__(self):
        super().__init__('reconstruction_node')

        self.encoder_samples = deque()
        self.points = []
        self.previous_angle = None
        self.sweep_direction = None

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.encoder_sub = self.create_subscription(
            JointState, '/encoder/state', self.encoder_callback, 100
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2, '/cloud', 10
        )

    def encoder_callback(self, msg):
        if not msg.position:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        angle = msg.position[0]

        self.encoder_samples.append((stamp, angle))

        # Detect reversal and publish the completed sweep
        if self.previous_angle is not None:
            delta = angle - self.previous_angle

            if abs(delta) > 1e-6:
                direction = 1 if delta > 0 else -1

                if self.sweep_direction is None:
                    self.sweep_direction = direction

                elif direction != self.sweep_direction:
                    self.publish_cloud(msg.header)
                    self.points = []
                    self.sweep_direction = direction

        self.previous_angle = angle

        # Keep a short history for interpolation
        while len(self.encoder_samples) > 2:
            if stamp - self.encoder_samples[0][0] > 2.0:
                self.encoder_samples.popleft()
            else:
                break

    def angle_at(self, timestamp):
        samples = list(self.encoder_samples)

        for i in range(len(samples) - 1):
            t1, a1 = samples[i]
            t2, a2 = samples[i + 1]

            if t1 <= timestamp <= t2:
                if t2 == t1:
                    return a1

                fraction = (timestamp - t1) / (t2 - t1)
                return a1 + fraction * (a2 - a1)

        return None

    def scan_callback(self, msg):
        start_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            ray_time = start_time + i * msg.time_increment
            phi = self.angle_at(ray_time)

            if phi is None:
                continue

            theta = msg.angle_min + i * msg.angle_increment

            local_x = r * math.cos(theta)
            z = 1.5 + r * math.sin(theta)

            x = local_x * math.cos(phi)
            y = local_x * math.sin(phi)

            self.points.append((x, y, z))

    def publish_cloud(self, header):
        if not self.points:
            return

        header.frame_id = 'world'

        cloud = point_cloud2.create_cloud_xyz32(
            header, self.points
        )

        self.cloud_pub.publish(cloud)

        self.get_logger().info(
            f'Published sweep with {len(self.points)} points'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReconstructionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()