# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import sys

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


class Printer(Node):
    def __init__(self):
        super().__init__("printer")
        self.__subscription = self.create_subscription(
            String,
            "/hello",
            self.hello_callback,
            qos_profile=QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )
        self.__timeout = self.create_timer(1, self.timeout_callback)

    def hello_callback(self, msg):
        self.__timeout.cancel()
        self.get_logger().info(msg.data)
        rclpy.shutdown()

    def timeout_callback(self):
        self.get_logger().error("printer did not receive a message")
        sys.exit(1)


def main(args=None):
    try:
        with rclpy.init(args=args):
            printer = Printer()
            rclpy.spin(printer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
