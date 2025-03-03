# -*- coding: utf-8 -*-
# Copyright 2025 ZiouHU
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
import rclpy
from rclpy.node import Node
from sine_wave_py.sine_wave_publisher import SineWavePublisher


class TestSineWavePublisher(unittest.TestCase):
    """Test cases for SineWavePublisher."""

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
        """Test that the publisher initializes without errors."""
        # Create a test node
        node = Node("test_node")

        # Set the parameters
        node.declare_parameter("amplitude", 1.0)
        node.declare_parameter("angular_frequency", 1.0)
        node.declare_parameter("phase", 0.0)
        node.declare_parameter("publisher_frequency", 10.0)

        # Check if SineWavePublisher initializes without errors
        try:
            publisher = SineWavePublisher(node)
            self.assertIsNotNone(publisher)
        except Exception as e:
            self.fail(f"SineWavePublisher initialization raised exception: {e}")
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
