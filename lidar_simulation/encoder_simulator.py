import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EncoderSimulator(Node):
    def __init__(self):
        super().__init__('encoder_simulator')

        self.publisher = self.create_publisher(
            JointState, '/encoder/state', 10
        )

        self.angle = 0.0
        self.direction = 1.0
        self.speed = 0.5  # radians per second
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.01, self.publish_angle)

    def publish_angle(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.angle += self.direction * self.speed * dt

        if self.angle >= 2.0 * math.pi:
            self.angle = 2.0 * math.pi
            self.direction = -1.0
        elif self.angle <= 0.0:
            self.angle = 0.0
            self.direction = 1.0

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ['hub']
        msg.position = [self.angle]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()