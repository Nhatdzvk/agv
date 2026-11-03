#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class LiftGazeboController(Node):

    def __init__(self):
        super().__init__("lift_gazebo_controller")

        # User command topic.
        self.command_subscription = self.create_subscription(
            Float64,
            "/lift_command",
            self.command_callback,
            10,
        )

        # Main vertical lift command.
        self.lift_publisher = self.create_publisher(
            Float64,
            "/lift_position",
            10,
        )

        # Four scissor joint command publishers.
        self.scissor_publishers = [
            self.create_publisher(
                Float64,
                "/scissor_left_a_position",
                10,
            ),
            self.create_publisher(
                Float64,
                "/scissor_left_b_position",
                10,
            ),
            self.create_publisher(
                Float64,
                "/scissor_right_a_position",
                10,
            ),
            self.create_publisher(
                Float64,
                "/scissor_right_b_position",
                10,
            ),
        ]

        self.target_lift = 0.0

        # Geometry from the OpenSCAD model, converted to meters.
        self.horizontal_span = 0.270
        self.base_vertical_difference = 0.042

        # Publish continuously so Gazebo controllers keep their targets.
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_commands,
        )

        self.get_logger().info(
            "Lift Gazebo controller started."
        )

        self.get_logger().info(
            "Send commands to /lift_command."
        )


    def command_callback(self, message):
        if not math.isfinite(message.data):
            return

        self.target_lift = max(
            0.0,
            min(
                0.180,
                message.data,
            ),
        )

        self.get_logger().info(
            f"Lift target = {self.target_lift:.3f} m"
        )


    def publish_commands(self):
        # Main platform position.
        lift_message = Float64()
        lift_message.data = self.target_lift

        self.lift_publisher.publish(
            lift_message
        )

        # Scissor angle follows the same lift position.
        vertical_difference = (
            self.base_vertical_difference +
            self.target_lift
        )

        scissor_angle = math.atan2(
            vertical_difference,
            self.horizontal_span,
        )

        angle_message = Float64()
        angle_message.data = scissor_angle

        for publisher in self.scissor_publishers:
            publisher.publish(
                angle_message
            )


def main(args=None):
    rclpy.init(args=args)

    node = LiftGazeboController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()