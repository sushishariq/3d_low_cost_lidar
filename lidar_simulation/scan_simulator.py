import math
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState


class ScanSimulator(Node):
    def __init__(self):
        super().__init__('scan_simulator')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.encoder_sub = self.create_subscription(
            JointState, '/encoder/state', self.encoder_callback, 100
        )

        self.encoder_samples = deque()
        self.scan_time = 0.1
        self.num_rays = 360

        self.timer = self.create_timer(
            self.scan_time, self.publish_scan
        )

    def encoder_callback(self, msg):
        if not msg.position:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.encoder_samples.append((stamp, msg.position[0]))

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

    def publish_scan(self):
        if len(self.encoder_samples) < 2:
            return

        now = self.get_clock().now()
        start_time = now.nanoseconds * 1e-9 - self.scan_time

        msg = LaserScan()
        msg.header.stamp.sec = int(start_time)
        msg.header.stamp.nanosec = int(
            (start_time - int(start_time)) * 1e9
        )
        msg.header.frame_id = 'laser'

        msg.angle_min = -math.pi
        msg.angle_increment = math.radians(1)
        msg.angle_max = (
            msg.angle_min + (self.num_rays - 1) * msg.angle_increment
        )

        msg.time_increment = self.scan_time / self.num_rays
        msg.scan_time = self.scan_time
        msg.range_min = 0.15
        msg.range_max = 12.0

        origin = (0.0, 0.0, 1.5)
        bounds = [(-5.0, 5.0), (-4.0, 4.0), (0.0, 3.0)]

        ranges = []

        for i in range(self.num_rays):
            ray_time = start_time + i * msg.time_increment
            phi = self.angle_at(ray_time)

            if phi is None:
                ranges.append(float('inf'))
                continue

            theta = msg.angle_min + i * msg.angle_increment

            direction = (
                math.cos(theta) * math.cos(phi),
                math.cos(theta) * math.sin(phi),
                math.sin(theta),
            )

            distances = []

            for axis in range(3):
                component = direction[axis]

                if abs(component) < 1e-9:
                    continue

                low, high = bounds[axis]
                boundary = high if component > 0 else low
                distance = (boundary - origin[axis]) / component

                if distance > 0:
                    distances.append(distance)

            ranges.append(min(distances) if distances else float('inf'))

        msg.ranges = ranges
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()