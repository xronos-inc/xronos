# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String


class Hello(Node):
    def __init__(self):
        super().__init__("hello")
        self.__publisher = self.create_publisher(
            String,
            "/hello",
            qos_profile=QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )
        self.__startup = self.create_timer(0.1, self.startup_callback)

    def startup_callback(self):
        msg = String()
        msg.data = "Hello, World!"
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.__publisher.publish(msg)
        self.__startup.cancel()
        rclpy.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            hello = Hello()
            rclpy.spin(hello)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
