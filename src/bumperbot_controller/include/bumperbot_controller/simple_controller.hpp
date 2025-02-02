#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>


class SimpleController : public rclcpp::Node
{
    public: 
        SimpleController(const std::string & name);

    private:
        void velCallback(const geometry_msgs::msg::TwistStamped & msg);

        void jointCallback(const sensor_msgs::msg::JointState & msg);
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;


        double wheel_radius;
        double wheel_separation;
        Eigen::Matrix2d speed_conversion;

        double left_wheel_prev_pos;
        double right_wheel_prev_pos;
        rclcpp::Time prev_time;

        double x;
        double y;
        double theta;

        nav_msgs::msg::Odometry odom_msg;

        std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
        geometry_msgs::msg::TransformStamped transform_stamped;


};

#endif