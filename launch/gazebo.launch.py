#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "factory_delivery_amr_description"

    package_share = get_package_share_directory(
        package_name
    )

    ros_gz_sim_share = get_package_share_directory(
        "ros_gz_sim"
    )


    # ========================================================================
    # FILE PATHS
    # ========================================================================

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "factory_delivery_amr.urdf.xacro",
    )

    world_file = os.path.join(
        package_share,
        "worlds",
        "factory_world.sdf",
    )

    bridge_file = os.path.join(
        package_share,
        "config",
        "bridge.yaml",
    )

    generated_urdf = (
        "/tmp/factory_delivery_amr_gazebo.urdf"
    )


    # ========================================================================
    # GENERATE URDF FROM XACRO
    # ========================================================================

    generate_urdf = ExecuteProcess(
        cmd=[
            "xacro",
            xacro_file,
            "-o",
            generated_urdf,
        ],
        output="screen",
    )


    # ========================================================================
    # START GAZEBO FORTRESS SERVER AND GUI
    #
    # -r:
    #   Start the simulation immediately.
    #
    # -v 4:
    #   Print detailed Gazebo diagnostic output.
    # ========================================================================

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": (
                "-r -v 4 " +
                world_file
            ),
            "on_exit_shutdown": "true",
        }.items(),
    )


    # ========================================================================
    # ROS 2 TO GAZEBO BRIDGE
    #
    # Topics are configured in:
    # config/bridge.yaml
    #
    # Expected topics:
    # - /clock
    # - /cmd_vel
    # - /odom
    # - /lift_position
    # - /scissor_left_a_position
    # - /scissor_left_b_position
    # - /scissor_right_a_position
    # - /scissor_right_b_position
    # ========================================================================

    bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": bridge_file,
                        "use_sim_time": True,
                    }
                ],
            )
        ],
    )


    # ========================================================================
    # SPAWN FACTORY DELIVERY AMR
    #
    # The delay gives Gazebo enough time to load the world and create:
    #
    # /world/factory_world/create
    # ========================================================================

    spawn_robot = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_factory_delivery_amr",
                output="screen",
                arguments=[
                    "-world",
                    "factory_world",
                    "-file",
                    generated_urdf,
                    "-name",
                    "factory_delivery_amr",
                    "-allow_renaming",
                    "true",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.01",
                ],
            )
        ],
    )


    # ========================================================================
    # LIFT AND SCISSOR SYNCHRONIZATION CONTROLLER
    #
    # Input:
    #   /lift_command
    #
    # Outputs:
    #   /lift_position
    #   /scissor_left_a_position
    #   /scissor_left_b_position
    #   /scissor_right_a_position
    #   /scissor_right_b_position
    #
    # The node converts the requested lift height into the correct angle
    # for all four scissor-bar joints.
    # ========================================================================

    lift_gazebo_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package=package_name,
                executable="lift_gazebo_controller.py",
                name="lift_gazebo_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
            )
        ],
    )


    # ========================================================================
    # COMPLETE LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription(
        [
            generate_urdf,
            start_gazebo,
            bridge,
            spawn_robot,
            lift_gazebo_controller,
        ]
    )