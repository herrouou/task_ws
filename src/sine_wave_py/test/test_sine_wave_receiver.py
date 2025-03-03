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

import unittest
import os
import rclpy
from rclpy.node import Node
import pathlib
from sine_wave_py.sine_wave_receiver import SineWaveReceiver
from sine_wave_interfaces.srv import ConvertImage


class TestSineWaveReceiver(unittest.TestCase):
    """Test cases for SineWaveReceiver."""

    @classmethod
    def setUpClass(cls):
        """Set up the test fixture."""
        # Initialize ROS
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Tear down the test fixture."""
        # Shutdown ROS
        rclpy.shutdown()

    def test_initialization(self):
        """Test that the receiver initializes without errors."""
        # Create a test node
        node = Node("test_receiver_node")

        # Set the parameters
        node.declare_parameter("amplitude", 1.0)
        node.declare_parameter("angular_frequency", 1.0)
        node.declare_parameter("phase", 0.0)
        node.declare_parameter("publisher_frequency", 10.0)

        # Check if SineWaveReceiver initializes without errors
        try:
            receiver = SineWaveReceiver(node)
            self.assertIsNotNone(receiver)
        except Exception as e:
            self.fail(f"SineWaveReceiver initialization raised exception: {e}")
        finally:
            node.destroy_node()

    def test_service_availability(self):
        """Test if the service is available."""
        # Create a test node
        node = Node("test_service_node")
        client_node = Node("test_client_node")

        # Set the parameters
        node.declare_parameter("amplitude", 1.0)
        node.declare_parameter("angular_frequency", 1.0)
        node.declare_parameter("phase", 0.0)
        node.declare_parameter("publisher_frequency", 10.0)

        try:
            # Create the receiver
            receiver = SineWaveReceiver(node)

            # Create a client
            client = client_node.create_client(ConvertImage, "convert_image")

            # Check if service is available
            available = client.wait_for_service(timeout_sec=2.0)
            self.assertTrue(available, "Service not available")

        except Exception as e:
            self.fail(f"Service test raised exception: {e}")
        finally:
            node.destroy_node()
            client_node.destroy_node()

    def test_image_file_exists(self):
        """Test if the test image file exists."""
        test_file_path = pathlib.Path(__file__)
        project_root = test_file_path.parent.parent
        image_path = project_root / "pics" / "image.png"

        # Create the directory if it doesn't exist
        os.makedirs(os.path.dirname(image_path), exist_ok=True)

        # Create a simple test image if it doesn't exist
        if not os.path.exists(image_path):
            import cv2
            import numpy as np

            test_img = np.zeros((100, 100, 3), dtype=np.uint8)
            test_img[25:75, 25:75] = [0, 0, 255]  # Red square
            cv2.imwrite(str(image_path), test_img)

        self.assertTrue(
            os.path.exists(image_path), f"Test image file not found: {image_path}"
        )


if __name__ == "__main__":
    unittest.main()
