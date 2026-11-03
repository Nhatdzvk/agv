#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ScissorVisualizer(Node):

    def __init__(self):
        super().__init__("scissor_visualizer")

        # ====================================================================
        # PUBLISHER AND SUBSCRIBER
        # ====================================================================

        self.publisher = self.create_publisher(
            MarkerArray,
            "/lift_scissor_markers",
            10,
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )


        # ====================================================================
        # CURRENT LIFT POSITION
        # ====================================================================

        self.lift_position = 0.0

        # Used to avoid printing the same value repeatedly.
        self.last_reported_position = None


        # ====================================================================
        # SCISSOR GEOMETRY
        #
        # Dimensions converted from OpenSCAD millimeters to meters.
        # ====================================================================

        # OpenSCAD:
        # lift_center_x = -15 mm
        self.lift_center_x = -0.015

        # OpenSCAD:
        # chassis_top_z1 = 256 mm
        # scissor_lower_z = chassis_top_z1 + 12 mm
        #
        # 256 + 12 = 268 mm
        self.lower_pivot_z = 0.268

        # OpenSCAD lowered platform:
        #
        # lift_lowered_deck_z = 330 mm
        # lift_deck_thickness = 12 mm
        #
        # deck_bottom_z = 330 - 12 = 318 mm
        #
        # scissor_upper_z = deck_bottom_z - 8
        # scissor_upper_z = 318 - 8 = 310 mm
        self.upper_pivot_base_z = 0.310

        # OpenSCAD horizontal pivot offset:
        # 135 mm
        self.x_offset = 0.135

        # OpenSCAD side positions:
        #
        # lift_rail_y - 38 = 150 - 38 = 112 mm
        # lift_rail_y - 52 = 150 - 52 = 98 mm
        self.side_y_a = 0.112
        self.side_y_b = 0.098

        # OpenSCAD scissor bar diameter:
        # 16 mm
        self.bar_diameter = 0.016


        # ====================================================================
        # MARKER UPDATE TIMER
        #
        # Publish markers at 30 Hz.
        # ====================================================================

        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_markers,
        )

        self.get_logger().info(
            "Scissor visualizer started."
        )

        self.get_logger().info(
            "Listening to /joint_states."
        )

        self.get_logger().info(
            "Publishing to /lift_scissor_markers."
        )


    # ========================================================================
    # JOINT STATE CALLBACK
    # ========================================================================

    def joint_state_callback(self, message):

        # Ignore messages that do not contain lift_joint.
        try:
            index = message.name.index("lift_joint")
        except ValueError:
            return

        # Protect against malformed JointState messages.
        if index >= len(message.position):
            return

        lift_position = message.position[index]

        # Ignore NaN or infinite values.
        if not math.isfinite(lift_position):
            return

        # Clamp the lift position to the physical travel range.
        self.lift_position = max(
            0.0,
            min(
                0.180,
                lift_position,
            ),
        )

        # Print only when the value actually changes.
        rounded_position = round(
            self.lift_position,
            3,
        )

        if rounded_position != self.last_reported_position:
            self.get_logger().info(
                f"lift_joint = {rounded_position:.3f} m"
            )

            self.last_reported_position = rounded_position


    # ========================================================================
    # QUATERNION FROM Y-AXIS ROTATION
    # ========================================================================

    @staticmethod
    def quaternion_from_y_rotation(angle):
        half_angle = angle / 2.0

        return (
            0.0,
            math.sin(half_angle),
            0.0,
            math.cos(half_angle),
        )


    # ========================================================================
    # CREATE ONE SCISSOR BAR MARKER
    #
    # RViz cylinder length is along its local Z axis.
    # The cylinder is rotated so local Z points from start to end.
    # ========================================================================

    def create_bar(
        self,
        marker_id,
        start_point,
        end_point,
    ):
        sx, sy, sz = start_point
        ex, ey, ez = end_point

        dx = ex - sx
        dy = ey - sy
        dz = ez - sz

        length = math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "lift_scissor"
        marker.id = marker_id

        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD


        # ====================================================================
        # POSITION AT THE MIDPOINT BETWEEN BOTH PIVOTS
        # ====================================================================

        marker.pose.position.x = (
            sx + ex
        ) / 2.0

        marker.pose.position.y = (
            sy + ey
        ) / 2.0

        marker.pose.position.z = (
            sz + ez
        ) / 2.0


        # ====================================================================
        # ORIENTATION
        #
        # The bars lie in the X-Z plane.
        # Rotate the cylinder around Y so its local Z axis aligns with
        # the vector from the lower pivot to the upper pivot.
        # ====================================================================

        angle_y = math.atan2(
            dx,
            dz,
        )

        qx, qy, qz, qw = self.quaternion_from_y_rotation(
            angle_y
        )

        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw


        # ====================================================================
        # SIZE
        # ====================================================================

        marker.scale.x = self.bar_diameter
        marker.scale.y = self.bar_diameter
        marker.scale.z = length


        # ====================================================================
        # COLOR
        #
        # Same orange-brown color as the OpenSCAD scissor bars.
        # ====================================================================

        marker.color.r = 0.82
        marker.color.g = 0.48
        marker.color.b = 0.08
        marker.color.a = 1.0

        # Keep the markers locked to base_link.
        marker.frame_locked = True

        return marker


    # ========================================================================
    # PUBLISH FOUR SCISSOR BARS
    #
    # Marker 0: Left side, diagonal A
    # Marker 1: Left side, diagonal B
    # Marker 2: Right side, diagonal A
    # Marker 3: Right side, diagonal B
    # ========================================================================

    def publish_markers(self):

        # Upper pivot moves vertically with lift_joint.
        upper_z = (
            self.upper_pivot_base_z +
            self.lift_position
        )

        x_left = (
            self.lift_center_x -
            self.x_offset
        )

        x_right = (
            self.lift_center_x +
            self.x_offset
        )

        marker_array = MarkerArray()


        # ====================================================================
        # LEFT-SIDE BAR A
        #
        # Lower-left pivot to upper-right pivot.
        # ====================================================================

        marker_array.markers.append(
            self.create_bar(
                marker_id=0,
                start_point=(
                    x_left,
                    self.side_y_a,
                    self.lower_pivot_z,
                ),
                end_point=(
                    x_right,
                    self.side_y_a,
                    upper_z,
                ),
            )
        )


        # ====================================================================
        # LEFT-SIDE BAR B
        #
        # Lower-right pivot to upper-left pivot.
        # ====================================================================

        marker_array.markers.append(
            self.create_bar(
                marker_id=1,
                start_point=(
                    x_right,
                    self.side_y_b,
                    self.lower_pivot_z,
                ),
                end_point=(
                    x_left,
                    self.side_y_b,
                    upper_z,
                ),
            )
        )


        # ====================================================================
        # RIGHT-SIDE BAR A
        #
        # Lower-left pivot to upper-right pivot.
        # ====================================================================

        marker_array.markers.append(
            self.create_bar(
                marker_id=2,
                start_point=(
                    x_left,
                    -self.side_y_a,
                    self.lower_pivot_z,
                ),
                end_point=(
                    x_right,
                    -self.side_y_a,
                    upper_z,
                ),
            )
        )


        # ====================================================================
        # RIGHT-SIDE BAR B
        #
        # Lower-right pivot to upper-left pivot.
        # ====================================================================

        marker_array.markers.append(
            self.create_bar(
                marker_id=3,
                start_point=(
                    x_right,
                    -self.side_y_b,
                    self.lower_pivot_z,
                ),
                end_point=(
                    x_left,
                    -self.side_y_b,
                    upper_z,
                ),
            )
        )

        self.publisher.publish(
            marker_array
        )


def main(args=None):
    rclpy.init(args=args)

    node = ScissorVisualizer()

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