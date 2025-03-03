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

#include "sine_wave_cpp/sine_wave_parameters.hpp"

#include <rclcpp/rclcpp.hpp>

SineWavePublisher::SineWavePublisher(rclcpp::Node::SharedPtr node, const sine_wave::Params & params)
: node_(node),
  amplitude_(params.amplitude),
  angular_frequency_(params.angular_frequency),
  phase_(params.phase),
  frequency_(params.publisher_frequency),
  time_(0.0)
{
  param_listener_ = std::make_shared<sine_wave::ParamListener>(node_);

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

  // Create a timer for checking parameter updates (e.g., every 1 second)
  update_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(0.3), std::bind(&SineWavePublisher::updateParamsCallback, this));

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
  // Lock mutex for safe access to shared parameters.
  // std::lock_guard<std::mutex> lock(mutex_);

  // Calculate the sine wave using current parameters
  double sine = amplitude_ * std::sin(angular_frequency_ * time_ + phase_);

  // Create and publish the message
  sine_wave_cpp::msg::Signal msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.data = sine;
  publisher_->publish(msg);

  // Increment time based on the current publisher frequency
  double dt = 1.0 / frequency_;
  time_ += dt;
}

void SineWavePublisher::updateParamsCallback()
{
  // Check if parameters have been updated via the parameter listener
  if (param_listener_->is_old(params_)) {
    // Retrieve new parameters
    auto new_params = param_listener_->get_params();

    // Update internal state for amplitude, angular frequency, and phase
    amplitude_ = new_params.amplitude;
    angular_frequency_ = new_params.angular_frequency;
    phase_ = new_params.phase;

    // If publisher frequency has changed, update frequency and recreate the timer
    if (frequency_ != new_params.publisher_frequency) {
      frequency_ = new_params.publisher_frequency;
      timer_->cancel();
      auto period = std::chrono::duration<double>(1.0 / frequency_);
      timer_ = node_->create_wall_timer(period, std::bind(&SineWavePublisher::timerCallback, this));
    }

    // Store the new parameters as current parameters
    params_ = new_params;
    RCLCPP_INFO(
      node_->get_logger(),
      "Parameters updated dynamically:\n"
      "Publisher frequency: %.2f Hz\n"
      "Amplitude: %.2f\n"
      "Angular frequency: %.2f\n"
      "Phase: %.2f",
      frequency_, amplitude_, angular_frequency_, phase_);
  }
}
