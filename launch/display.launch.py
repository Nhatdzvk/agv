#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "factory_delivery_amr_description"

    package_share = get_package_share_directory(
        package_name
    )

    default_xacro_file = os.path.join(
        package_share,
        "urdf",
        "factory_delivery_amr.urdf.xacro",
    )


    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================

    model_argument = DeclareLaunchArgument(
        name="model",
        default_value=default_xacro_file,
        description="Absolute path to the robot Xacro file",
    )

    use_sim_time_argument = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )


    # ========================================================================
    # ROBOT DESCRIPTION
    # ========================================================================

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
            ]
        ),
        value_type=str,
    )


    # ========================================================================
    # ROBOT STATE PUBLISHER
    # Publishes the robot TF tree from robot_description and joint_states.
    # ========================================================================

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ],
    )


    # ========================================================================
    # JOINT STATE PUBLISHER GUI
    # Provides sliders for:
    # - left_wheel_joint
    # - right_wheel_joint
    # - lift_joint
    # ========================================================================

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ],
    )


    # ========================================================================
    # SCISSOR LIFT VISUALIZER
    #
    # Reads:
    #   /joint_states
    #
    # Publishes:
    #   /lift_scissor_markers
    #
    # The four scissor bars change position, angle and length according
    # to the current lift_joint position.
    # ========================================================================

    scissor_visualizer_node = Node(
        package=package_name,
        executable="scissor_visualizer.py",
        name="scissor_visualizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ],
    )


    # ========================================================================
    # RVIZ2
    # ========================================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration(
                    "use_sim_time"
                ),
            }
        ],
    )


    # ========================================================================
    # COMPLETE LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription(
        [
            model_argument,
            use_sim_time_argument,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            scissor_visualizer_node,
            rviz_node,
        ]
    )