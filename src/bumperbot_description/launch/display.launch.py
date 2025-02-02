from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
import os


def generate_launch_description():
    # Absolute path to the robot model (URDF file)
    model_path = "/home/aca/ros2_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro"

    # Absolute path to the RViz configuration file
    rviz_config_path = "/home/aca/ros2_ws/src/bumperbot_description/rviz/display.rviz"

    # Declare the robot model argument (can still allow overriding, if needed)
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=model_path,
        description="Absolute path to robot URDF file"
    )

    # Define the robot description parameter
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Node for the robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Node for the joint state publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Node for RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )

    # Return the launch description
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
