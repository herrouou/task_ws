#!/usr/bin/env python3
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
from sine_wave_py.sine_wave_publisher import SineWavePublisher


class SineWavePublisherNode(Node):
    """Node for publishing sine wave data."""

    def __init__(self):
        """Initialize the node."""
        super().__init__("sine_wave_publisher")
        self.publisher = SineWavePublisher(self)
        self.get_logger().info(
            "Sine Wave Publisher node is running and publishing msg on 'sine_wave' topic."
        )


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
