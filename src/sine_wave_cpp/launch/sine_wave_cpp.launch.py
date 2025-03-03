#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import launch
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

current_dir = os.path.dirname(__file__)
plot_config = os.path.join(
    current_dir, "..", "src", "sine_wave_cpp", "config", "plot.xml"
)


def generate_launch_description():
    plot_config = os.path.join(
        get_package_share_directory("sine_wave_cpp"), "config", "plot.xml"
    )
    return LaunchDescription(
        [
            # run the publisher
            Node(
                package="sine_wave_cpp",
                executable="sine_wave_publisher_node",
                name="sine_wave_publisher",
                output="screen",
            ),
            # run the receiver
            Node(
                package="sine_wave_cpp",
                executable="sine_wave_receiver_node",
                name="sine_wave_receiver",
                output="screen",
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
