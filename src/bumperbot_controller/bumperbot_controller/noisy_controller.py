#!/bin/env python3

import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import TransformStamped
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value 
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value 

        self.get_logger().info("Using wheel radius %f" % self.wheel_radius)
        self.get_logger().info("Using wheel separation %f" % self.wheel_separation)

        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time = self.get_clock().now()

        self.x= 0.0
        self.y = 0.0
        self.theta = 0.0


        self.joint_sub = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub = self.create_publisher(Odometry, "bumperbot_controller/odom_noisy", 10)

        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint_ekf"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.br = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint_noisy"


    def jointCallback(self, msg):
        wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[0] + np.random.normal(0, 0.005)
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos
        dt = Time.from_msg(msg.header.stamp) - self.prev_time

        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear = (self.wheel_radius * fi_right + self.wheel_radius * fi_left) / 2
        angular = (self.wheel_radius * fi_right - self.wheel_radius * fi_left) / self.wheel_separation

        d_s = (self.wheel_radius * dp_right + self.wheel_radius * dp_left) / 2
        d_theta = (self.wheel_radius * dp_right - self.wheel_radius * dp_left) / self.wheel_separation
        self.theta += d_theta 
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        q = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular

        self.transform_stamped.transform.translation.x = self.x 
        self.transform_stamped.transform.translation.y = self.y 
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(self.odom_msg)
        self.br.sendTransform(self.transform_stamped)

        self.get_logger().info("linear : %f, angular : %f" % (linear, angular))
        self.get_logger().info("x : %f, y : %f, theta: %f" % (self.x, self.y, self.theta))


def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
