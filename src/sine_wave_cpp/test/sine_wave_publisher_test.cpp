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

#include "sine_wave_cpp/sine_wave_publisher.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sine_wave_cpp/sine_wave_parameters.hpp>

#include <gtest/gtest.h>  // NOLINT(build/include_order)

TEST(SineWavePublisherTest, Initialization)
{
  rclcpp::init(0, nullptr);

  // create a node
  auto node = std::make_shared<rclcpp::Node>("test_node");

  // set some values
  sine_wave::Params params;
  params.amplitude = 1.0;
  params.angular_frequency = 1.0;
  params.phase = 0.0;
  params.publisher_frequency = 10.0;

  // check the SineWavePublisher
  EXPECT_NO_THROW({ SineWavePublisher publisher(node, params); });

  rclcpp::shutdown();
}
