#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class ModeToggle(Node):
    def __init__(self):
        super().__init__("keyboard_toggle_node")

        # Service client for toggling autonomous/manual mode
        self.cli = self.create_client(Trigger, "/toggle_autonomous")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /toggle_autonomous service...")

        # Subscriber to receive actual mode from stack
        self.subscription = self.create_subscription(
            Bool,
            "/autonomous_mode_state",
            self.mode_callback,
            10
        )

        self.current_mode = False
        self.get_logger().info("Press Z to toggle mode. Press Q to quit.")

        self.settings = termios.tcgetattr(sys.stdin)

    def mode_callback(self, msg):
        self.current_mode = msg.data
        mode_str = "AUTONOMOUS" if self.current_mode else "MANUAL"
        self.get_logger().info(f"Mode updated: {mode_str}")

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        while rclpy.ok():
            key = self.read_key().lower()

            if key == "z":
                req = Trigger.Request()
                future = self.cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)

                if future.result():
                    self.get_logger().info(future.result().message)

            elif key == "q":
                self.get_logger().info("Exiting keyboard control")
                break


def main():
    rclpy.init()
    node = ModeToggle()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
