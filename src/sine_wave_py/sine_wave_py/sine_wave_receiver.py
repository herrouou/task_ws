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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2
import numpy as np
from cv_bridge import CvBridge
from sine_wave_cpp.msg import Signal
from sine_wave_cpp.srv import ConvertImage
from sine_wave_py.sine_wave_parameters import sine_wave_parameters


class SineWaveReceiver:
    """Class to receive sine wave data and provide image conversion service with dynamic parameter updates."""

    def __init__(self, node: Node):
        """
        Initialize the sine wave receiver.

        Args:
            node: ROS2 node
        """
        self.node = node
        self.param_listener = sine_wave_parameters.ParamListener(self.node)
        self.params = self.param_listener.get_params()  # get initial parameters

        # Get parameters
        self.amplitude = self.params.amplitude
        self.angular_frequency = self.params.angular_frequency
        self.phase = self.params.phase
        self.frequency = self.params.publisher_frequency

        # Validate parameters
        self.validate_parameters()

        # Create callback groups
        self.subscription_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # Create subscription
        self.subscription = self.node.create_subscription(
            Signal,
            "sine_wave",
            self.sine_wave_callback,
            10,
            callback_group=self.subscription_cb_group,
        )

        # Create service
        self.service = self.node.create_service(
            ConvertImage,
            "convert_image",
            self.convert_image_service,
            callback_group=self.service_cb_group,
        )

        # Initialize bridge for OpenCV conversion
        self.bridge = CvBridge()

        # Create a timer for checking parameter updates (every 0.3 seconds)
        self.update_timer = self.node.create_timer(0.3, self.update_params_callback)

        self.node.get_logger().info(
            "SineWaveReceiver node with custom Service is ready."
        )

    def declare_parameters(self):
        """Declare all parameters."""
        self.node.declare_parameter("amplitude", 1.0)
        self.node.declare_parameter("angular_frequency", 1.0)
        self.node.declare_parameter("phase", 0.0)
        self.node.declare_parameter("publisher_frequency", 10.0)

    def validate_parameters(self):
        """Validate the parameters and set defaults if needed."""
        # Validate frequency
        if self.frequency < 0.001 or self.frequency > 99999.9:
            self.node.get_logger().error(
                f"Invalid publisher_frequency ({self.frequency}). "
                "Must be within [0.001, 99999.9]. Using fallback value 10.0 Hz."
            )
            self.frequency = 10.0

        # Validate amplitude
        if self.amplitude < 0.0000001 or self.amplitude > 99999.9:
            self.node.get_logger().error(
                f"Invalid amplitude ({self.amplitude}). "
                "Must be within [0.0000001, 99999.9]. Using fallback value 1.0."
            )
            self.amplitude = 1.0

        # Validate angular_frequency
        if self.angular_frequency < 0.000001 or self.angular_frequency > 99999.9:
            self.node.get_logger().error(
                f"Invalid angular_frequency ({self.angular_frequency}). "
                "Must be within [0.000001, 99999.9]. Using fallback value 1.0."
            )
            self.angular_frequency = 1.0

        # Validate phase
        if self.phase < -6.283184 or self.phase > 6.283184:
            self.node.get_logger().error(
                f"Invalid phase ({self.phase}). "
                "Must be within [-6.283184, 6.283184]. Using fallback value 0.0."
            )
            self.phase = 0.0

    def sine_wave_callback(self, msg):
        """
        Process the received sine wave message.

        Args:
            msg: Received Signal message
        """
        sine_value = msg.data
        stamp = msg.header.stamp
        self.node.get_logger().info(
            f"Received sine wave value: {sine_value}, timestamp: {stamp.sec}.{stamp.nanosec}"
        )

    def convert_image_service(self, request, response):
        """
        Service to convert a color image to grayscale.

        Args:
            request: Service request containing file path
            response: Service response to contain grayscale image

        Returns:
            Response with grayscale image
        """
        self.node.get_logger().info(
            f"Service callback invoked. file_path: {request.file_path}"
        )

        # Read the image using OpenCV
        color_img = cv2.imread(request.file_path, cv2.IMREAD_COLOR)
        if color_img is None:
            self.node.get_logger().error(
                f"Failed to read image from path: {request.file_path}"
            )
            return response

        self.node.get_logger().info("Image successfully read.")

        # Convert to grayscale
        gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # Convert the image to a ROS Image message (encoding "mono8")
        response.grayscale_image = self.bridge.cv2_to_imgmsg(gray_img, encoding="mono8")

        self.node.get_logger().info("Successfully transformed image to gray image!")

        # Visualize the images side by side
        gray_bgr = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
        concatenated = np.hstack((color_img, gray_bgr))

        cv2.namedWindow("origin image | gray image", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("origin image | gray image", concatenated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return response

    def update_params_callback(self):
        """
        Timer callback that checks for parameter updates.

        If new parameters are detected via the parameter listener, update the internal state.
        """
        if self.param_listener.is_old(self.params):
            # Retrieve new parameters
            new_params = self.param_listener.get_params()

            # Update internal state for publisher_frequency, amplitude, angular frequency, and phase
            self.frequency = new_params.publisher_frequency
            self.amplitude = new_params.amplitude
            self.angular_frequency = new_params.angular_frequency
            self.phase = new_params.phase

            # Store the new parameters as current parameters
            self.params = new_params

            self.node.get_logger().info(
                f"Parameters updated dynamically:\n"
                f"Publisher frequency: {self.frequency:.2f} Hz\n"
                f"Amplitude: {self.amplitude:.2f}\n"
                f"Angular frequency: {self.angular_frequency:.2f}\n"
                f"Phase: {self.phase:.2f}"
            )
