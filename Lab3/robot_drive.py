#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class RobotHexagon(Node):

    def __init__(self):
        super().__init__('robot_hexagon')

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, '/robot_11/cmd_vel', 10)
        self.create_subscription(Odometry, '/robot_11/odom', self.odom_callback, 10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

        # Motion Parameters
        self.side_length = 1.0
        self.number_of_sides = 6
        self.turn_angle = math.radians(60.0)

        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.distance_tolerance = 0.02
        self.angle_tolerance = math.radians(2.0)

        # Pose Variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Control State
        self.current_side = 0
        self.motion_state = "forward"

        self.start_x = 0.0
        self.start_y = 0.0
        self.target_yaw = 0.0

        self.get_logger().info("Hexagon motion started.")


    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.x = position.x
        self.y = position.y
        self.yaw = self.get_yaw_from_quaternion(
            orientation.x, orientation.y,
            orientation.z, orientation.w
        )

        if not self.odom_received:
            self.start_x = self.x
            self.start_y = self.y
            self.odom_received = True


    def control_loop(self):

        if not self.odom_received:
            self.publish_velocity(0.0, 0.0)
            return

        if self.current_side >= self.number_of_sides:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info("Hexagon completed.")
            self.timer.cancel()
            return

        if self.motion_state == "forward":
            self.move_forward()
        else:
            self.rotate_robot()


    def move_forward(self):
        distance = math.sqrt(
            (self.x - self.start_x) ** 2 +
            (self.y - self.start_y) ** 2
        )

        if distance >= (self.side_length - self.distance_tolerance):
            self.publish_velocity(0.0, 0.0)
            self.target_yaw = self.normalize_angle(self.yaw + self.turn_angle)
            self.motion_state = "turn"
            return

        self.publish_velocity(self.linear_speed, 0.0)


    def rotate_robot(self):
        angle_error = self.normalize_angle(self.target_yaw - self.yaw)

        if abs(angle_error) <= self.angle_tolerance:
            self.publish_velocity(0.0, 0.0)

            self.current_side += 1
            self.start_x = self.x
            self.start_y = self.y
            self.motion_state = "forward"
            return

        direction = 1.0 if angle_error > 0 else -1.0
        self.publish_velocity(0.0, direction * self.angular_speed)

    def publish_velocity(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rplidar_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def get_yaw_from_quaternion(self, x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = RobotHexagon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
