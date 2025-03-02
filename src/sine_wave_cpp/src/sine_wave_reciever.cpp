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

#include "sine_wave_cpp/sine_wave_reciever.hpp"

#include "sine_wave_cpp/sine_wave_parameters.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <cv_bridge/cv_bridge.h>  // NOLINT(build/include_order)

#include <mutex>
#include <thread>

SineWaveReciever::SineWaveReciever(rclcpp::Node::SharedPtr node, const sine_wave::Params & params)
: node_(node),
  amplitude_(params.amplitude),
  angular_frequency_(params.angular_frequency),
  phase_(params.phase),
  frequency_(params.publisher_frequency)
{
  // validate frequency
  if (frequency_ < 0.001 || frequency_ > 99999.9) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid publisher_frequency (%f). Must be within [0.001, 99999.9]. Using fallback value "
      "10.0 Hz.",
      frequency_);
    frequency_ = 10.0;
  }

  // Validate amplitude
  if (amplitude_ < 0.0000001 || amplitude_ > 99999.9) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid amplitude (%f). Must be within [0.0000001, 99999.9]. Using fallback value 1.0.",
      amplitude_);
    amplitude_ = 1.0;
  }

  // Validate angular_frequency
  if (angular_frequency_ < 0.000001 || angular_frequency_ > 99999.9) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid angular_frequency (%f). Must be within [0.000001, 99999.9]. Using fallback value "
      "1.0.",
      angular_frequency_);
    angular_frequency_ = 1.0;
  }

  // Validate phase
  if (phase_ < -6.283184 || phase_ > 6.283184) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid phase (%f). Must be within [-6.283184, 6.283184]. Using fallback value 0.0.",
      phase_);
    phase_ = 0.0;
  }

  // define the callbackgroup to introduce the multi-thread, use MutuallyExclusive to make
  // subscription to be extra thread
  auto subscription_cb_group =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto service_cb_group =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // create subscription
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = subscription_cb_group;
  subscription_ = node_->create_subscription<sine_wave_cpp::msg::Signal>(
    "sine_wave", 10, std::bind(&SineWaveReciever::sinewaveCallback, this, std::placeholders::_1),
    sub_options);

  // create Service
  service_ = node_->create_service<sine_wave_cpp::srv::ConvertImage>(
    "convert_image",
    std::bind(
      &SineWaveReciever::convertImageService, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_default);

  RCLCPP_INFO(node_->get_logger(), "SineWaveReciever node with custom Service is ready.");
}

void SineWaveReciever::sinewaveCallback(const sine_wave_cpp::msg::Signal::SharedPtr msg)
{
  double sine_value = msg->data;
  auto stamp = msg->header.stamp;
  RCLCPP_INFO(
    node_->get_logger(), "Received sine wave value: %f, timestamp: %d.%u", sine_value, stamp.sec,
    stamp.nanosec);
}

void SineWaveReciever::convertImageService(
  const std::shared_ptr<sine_wave_cpp::srv::ConvertImage::Request> request,
  std::shared_ptr<sine_wave_cpp::srv::ConvertImage::Response> response)
{
  // 1. use opencv to read image
  RCLCPP_INFO(
    node_->get_logger(), "Service callback invoked. file_path: %s", request->file_path.c_str());

  cv::Mat color_img = cv::imread(request->file_path, cv::IMREAD_COLOR);
  if (color_img.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to read image from path: %s", request->file_path.c_str());
    response->grayscale_image = sensor_msgs::msg::Image();
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Image successfully read.");

  // 2. to gary image
  cv::Mat gray_img;
  cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);

  // 3. transform it to sensor_msgs::Image and encoding to "mono8"
  std_msgs::msg::Header header;
  header.stamp = node_->now();
  auto cv_bridge_img = cv_bridge::CvImage(header, "mono8", gray_img);
  sensor_msgs::msg::Image ros_img_msg = *cv_bridge_img.toImageMsg();

  // 4. set the response
  response->grayscale_image = ros_img_msg;

  RCLCPP_INFO(node_->get_logger(), "Successfully transform image to gray image!");

  // 5. visualize it
  cv::Mat gray_bgr;
  cv::cvtColor(gray_img, gray_bgr, cv::COLOR_GRAY2BGR);

  cv::Mat concatenated;
  cv::hconcat(color_img, gray_bgr, concatenated);

  cv::namedWindow("oringin image | gray image", cv::WINDOW_AUTOSIZE);
  cv::imshow("oringin image | gray image", concatenated);
  cv::waitKey(0);
  cv::destroyAllWindows();
  // {
  //   std::lock_guard<std::mutex> lock(g_img_mutex);
  //   concatenated.copyTo(g_img_to_display);
  //   g_new_image_available = true;
  // }
  // g_img_cv.notify_one();
}
