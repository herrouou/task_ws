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

#include "sine_wave_cpp/sine_wave_parameters.hpp"

SineWavePublisher::SineWavePublisher(rclcpp::Node::SharedPtr node, const sine_wave::Params & params)
: node_(node),
  amplitude_(params.amplitude),
  angular_frequency_(params.angular_frequency),
  phase_(params.phase),
  frequency_(params.publisher_frequency),
  time_(0.0)
{
  // validate frequency
  if (frequency_ < 0.001 || frequency_ > 99999.9) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid publisher_frequency (%f). Must be in [0.001, 99999.9]."
      "10.0 Hz.",
      frequency_);
    frequency_ = 10.0;
  }

  // Validate amplitude
  if (amplitude_ < 0.0000001 || amplitude_ > 99999.9) {
    RCLCPP_ERROR(
      node_->get_logger(), "Invalid amplitude (%f). Must be in [0.0000001, 99999.9].", amplitude_);
    amplitude_ = 1.0;
  }

  // Validate angular_frequency
  if (angular_frequency_ < 0.000001 || angular_frequency_ > 99999.9) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid angular_frequency (%f). Must be in [0.000001, 99999.9]."
      "1.0.",
      angular_frequency_);
    angular_frequency_ = 1.0;
  }

  // Validate phase
  if (phase_ < -6.283184 || phase_ > 6.283184) {
    RCLCPP_ERROR(
      node_->get_logger(), "Invalid phase (%f). Must be in [-6.283184, 6.283184].", phase_);
    phase_ = 0.0;
  }

  // Create a publisher on the topic "sine_wave"
  publisher_ = node_->create_publisher<sine_wave_cpp::msg::Signal>("sine_wave", 10);

  // Create a timer according to frequency
  auto period = std::chrono::duration<double>(1.0 / frequency_);
  timer_ = node_->create_wall_timer(period, std::bind(&SineWavePublisher::timerCallback, this));

  RCLCPP_INFO(
    node_->get_logger(),
    "SineWavePublisher initialized with:\n"
    "The publisher_frequency is %.2f Hz\n"
    "The amplitude is %.2f\n"
    "The angular_frequency is %.2f\n"
    "The phase is %.2f",
    frequency_, amplitude_, angular_frequency_, phase_);
}

void SineWavePublisher::timerCallback()
{
  // Calculate the sine wave
  double sine = amplitude_ * std::sin(angular_frequency_ * time_ + phase_);

  // Publish the sine wave
  sine_wave_cpp::msg::Signal msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.data = sine;
  publisher_->publish(msg);

  // Increment time
  double dt = 1.0 / frequency_;
  time_ += dt;
}
