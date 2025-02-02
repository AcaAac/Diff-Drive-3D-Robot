#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <functional>  // Ensure functional is included for std::bind
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

SimpleController::SimpleController(const std::string & name)
    : Node(name),
    left_wheel_prev_pos(0.0),
    right_wheel_prev_pos(0.0),
    x(0.0),
    y(0.0),
    theta(0.0)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel_radius: " << wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel_separation: " << wheel_separation);

    prev_time = get_clock()->now();

    wheel_cmd_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/bumperbot_controller/cmd_vel", 10,
        std::bind(&SimpleController::velCallback, this, _1));
    joint_sub = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1));
    
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10);
    speed_conversion << wheel_radius / 2, wheel_radius / 2, 
                        wheel_radius / wheel_separation, -wheel_radius / wheel_separation;

    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    transform_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_footprint";


    RCLCPP_INFO_STREAM(get_logger(), "The conversion matrix is: \n" << speed_conversion);
}

void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped & msg)
{
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);

    Eigen::Vector2d wheel_speed = speed_conversion.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    wheel_cmd_pub->publish(wheel_speed_msg);
}
void SimpleController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    double dp_left = msg.position.at(1) - left_wheel_prev_pos;
    double dp_right = msg.position.at(0) - right_wheel_prev_pos;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time;



    left_wheel_prev_pos = msg.position.at(1);
    right_wheel_prev_pos = msg.position.at(0);
    prev_time = msg_time;

    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    double linear = (wheel_radius * fi_right + wheel_radius * fi_left) / 2;
    double angular = (wheel_radius * fi_right - wheel_radius * fi_left) / wheel_separation;

    double d_s = (wheel_radius * dp_right + wheel_radius * dp_left) / 2.0;
    double d_theta = (wheel_radius * dp_right - wheel_radius * dp_left) / wheel_separation;

    theta += d_theta;
    x += d_s * cos(theta);
    y += d_s * sin(theta);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.angular.z = angular;

    transform_stamped.transform.translation.x = x; 
    transform_stamped.transform.translation.y = y; 
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    transform_stamped.header.stamp = get_clock()->now();


    odom_pub->publish(odom_msg);
    transform_broadcaster->sendTransform(transform_stamped);

    RCLCPP_INFO_STREAM(get_logger(), "Linear: " << linear << " Angular: " << angular);
    RCLCPP_INFO_STREAM(get_logger(), "x: " << x << " y: " << y << " theta: " << theta);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
