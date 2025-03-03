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

#ifndef SINE_WAVE_CPP__SINE_WAVE_PUBLISHER_HPP_
#define SINE_WAVE_CPP__SINE_WAVE_PUBLISHER_HPP_

#include "sine_wave_cpp/msg/signal.hpp"
#include "sine_wave_cpp/sine_wave_parameters.hpp"

#include <memory>  // NOLINT(build/include_order)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class SineWavePublisher
{
public:
  /**
   * @brief A SineWavePublisher object
   * @param node  Shared pointer of rclcpp::Node
   * @param params The sine wave's parameters generated by generate_parameter_library
   */
  SineWavePublisher(rclcpp::Node::SharedPtr node, const sine_wave::Params & params);

private:
  /**
   * @brief Timer callback that publishes the current sine wave value
   */
  void timerCallback();

  /**
   * @brief Timer callback to check for parameter updates.
   *
   * If new parameters are detected, internal state is updated accordingly.
   */
  void updateParamsCallback();

  // declare node, publisher and timer
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sine_wave_cpp::msg::Signal>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::shared_ptr<sine_wave::ParamListener> param_listener_;
  sine_wave::Params params_;

  // Sine wave parameters
  double amplitude_;
  double angular_frequency_;
  double phase_;
  double frequency_;

  // Time accumulator
  double time_;

  // Mutex to protect shared resources between timer callbacks.
  std::mutex mutex_;
};

#endif  // SINE_WAVE_CPP__SINE_WAVE_PUBLISHER_HPP_
