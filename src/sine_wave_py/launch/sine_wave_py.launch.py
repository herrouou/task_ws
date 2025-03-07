# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    plot_config = os.path.join(
        get_package_share_directory("sine_wave_py"), "config", "plot.xml"
    )
    return LaunchDescription(
        [
            # run the publisher
            Node(
                package="sine_wave_py",
                executable="sine_wave_publisher_node",
                name="sine_wave_publisher",
                output="screen",
                # parameters=[{
                #     'amplitude': 1.0,
                #     'angular_frequency': 1.0,
                #     'phase': 0.0,
                #     'publisher_frequency': 10.0
                # }]
            ),
            # run the receiver
            Node(
                package="sine_wave_py",
                executable="sine_wave_receiver_node",
                name="sine_wave_receiver",
                output="screen",
                # parameters=[{
                #     'amplitude': 1.0,
                #     'angular_frequency': 1.0,
                #     'phase': 0.0,
                #     'publisher_frequency': 10.0
                # }]
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "plotjuggler",
                    "plotjuggler",
                    "--layout",
                    plot_config,
                ],
                output="screen",
            ),
        ]
    )
