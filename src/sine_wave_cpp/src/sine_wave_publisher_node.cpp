/*
 * Copyright 2015 Ziou
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sine_wave_cpp/sine_wave_parameters.hpp"
#include "sine_wave_cpp/sine_wave_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a standard rclcpp::Node
  auto node = std::make_shared<rclcpp::Node>("sine_wave_publisher");

  // Create a parameter listener by using the generated library
  auto param_listener = std::make_shared<sine_wave::ParamListener>(node);

  auto params = param_listener->get_params();

  // Instantiate our SineWavePublisher
  auto sine_wave_publisher = std::make_shared<SineWavePublisher>(node, params);

  RCLCPP_INFO(
    node->get_logger(),
    "Sine Wave Publisher node is running and publishing msg on 'sine_wave' topic.");

  // Spin the node
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
