# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
        ]
    )
