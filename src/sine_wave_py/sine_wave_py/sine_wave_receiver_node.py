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
from rclpy.executors import MultiThreadedExecutor
from sine_wave_py.sine_wave_receiver import SineWaveReceiver


class SineWaveReceiverNode(Node):
    """Node for receiving sine wave data and providing image conversion service."""

    def __init__(self):
        """Initialize the node."""
        super().__init__("sine_wave_receiver")
        self.receiver = SineWaveReceiver(self)
        self.get_logger().info(
            "Sine Wave Receiver node is running and trying to receive msg on 'sine_wave' topic."
        )


def main(args=None):
    rclpy.init(args=args)
    node = SineWaveReceiverNode()

    # Create a multithreaded executor with 2 threads
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
