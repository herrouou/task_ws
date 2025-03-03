# -*- coding: utf-8 -*-
# Copyright 2025 ZiouHu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import math
from sine_wave_cpp.msg import Signal
from sine_wave_py.sine_wave_parameters import sine_wave_parameters


class SineWavePublisher:
    """Class to publish sine wave data with dynamic parameter updates."""

    def __init__(self, node: Node):
        """
        Initialize the sine wave publisher.

        Args:
            node: ROS2 node
        """
        self.node = node
        self.param_listener = sine_wave_parameters.ParamListener(self.node)
        self.params = self.param_listener.get_params()  # get initial parameters

        self.amplitude = self.params.amplitude
        self.angular_frequency = self.params.angular_frequency
        self.phase = self.params.phase
        self.frequency = self.params.publisher_frequency

        # Validate parameters
        self.validate_parameters()

        # Create publisher
        self.publisher = self.node.create_publisher(Signal, "sine_wave", 10)

        # Create a timer for publishing the sine wave signal
        period = 1.0 / self.frequency
        self.timer = self.node.create_timer(period, self.timer_callback)

        # Create a timer for checking parameter updates (every 0.3 seconds)
        self.update_timer = self.node.create_timer(0.3, self.update_params_callback)

        # Time accumulator
        self.time = 0.0

        self.node.get_logger().info(
            f"SineWavePublisher initialized with:\n"
            f"The publisher_frequency is {self.frequency:.2f} Hz\n"
            f"The amplitude is {self.amplitude:.2f}\n"
            f"The angular_frequency is {self.angular_frequency:.2f}\n"
            f"The phase is {self.phase:.2f}"
        )

    def validate_parameters(self):
        """Validate the parameters and set defaults if needed."""
        # Validate frequency
        if self.frequency < 0.001 or self.frequency > 99999.9:
            self.node.get_logger().error(
                f"Invalid publisher_frequency ({self.frequency}). "
                "Must be in [0.001, 99999.9]. Using fallback value 10.0 Hz."
            )
            self.frequency = 10.0

        # Validate amplitude
        if self.amplitude < 0.0000001 or self.amplitude > 99999.9:
            self.node.get_logger().error(
                f"Invalid amplitude ({self.amplitude}). "
                "Must be in [0.0000001, 99999.9]. Using fallback value 1.0."
            )
            self.amplitude = 1.0

        # Validate angular_frequency
        if self.angular_frequency < 0.000001 or self.angular_frequency > 99999.9:
            self.node.get_logger().error(
                f"Invalid angular_frequency ({self.angular_frequency}). "
                "Must be in [0.000001, 99999.9]. Using fallback value 1.0."
            )
            self.angular_frequency = 1.0

        # Validate phase
        if self.phase < -6.283184 or self.phase > 6.283184:
            self.node.get_logger().error(
                f"Invalid phase ({self.phase}). "
                "Must be in [-6.283184, 6.283184]. Using fallback value 0.0."
            )
            self.phase = 0.0

    def timer_callback(self):
        """
        Timer callback that publishes the current sine wave value.
        """
        # Calculate the sine wave using current parameters
        sine = self.amplitude * math.sin(
            self.angular_frequency * self.time + self.phase
        )

        # Create and publish the message
        msg = Signal()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = sine
        self.publisher.publish(msg)

        # Increment time based on the current publisher frequency
        dt = 1.0 / self.frequency
        self.time += dt

    def update_params_callback(self):
        """
        Timer callback that checks for parameter updates.

        If new parameters are available, update internal state.
        If the publisher frequency changes, cancel and recreate the publishing timer.
        """
        if self.param_listener.is_old(self.params):
            # Retrieve new parameters
            new_params = self.param_listener.get_params()

            # Update internal state for amplitude, angular frequency, and phase
            self.amplitude = new_params.amplitude
            self.angular_frequency = new_params.angular_frequency
            self.phase = new_params.phase

            # If publisher frequency has changed, update frequency and recreate
            # the timer
            if self.frequency != new_params.publisher_frequency:
                self.frequency = new_params.publisher_frequency
                self.timer.cancel()
                period = 1.0 / self.frequency
                self.timer = self.node.create_timer(period, self.timer_callback)

            # Store the new parameters as current parameters
            self.params = new_params

            self.node.get_logger().info(
                f"Parameters updated dynamically:\n"
                f"Publisher frequency: {self.frequency:.2f} Hz\n"
                f"Amplitude: {self.amplitude:.2f}\n"
                f"Angular frequency: {self.angular_frequency:.2f}\n"
                f"Phase: {self.phase:.2f}"
            )
