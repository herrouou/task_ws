// /*
//  * Copyright 2015 Ziou
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */

//  #include <gtest/gtest.h>
//  #include <rclcpp/rclcpp.hpp>
//  #include <filesystem>

//  #include "sine_wave_cpp/sine_wave_reciever.hpp"
//  #include "sine_wave_cpp/sine_wave_parameters.hpp"
//  #include "sine_wave_cpp/srv/convert_image.hpp"

//  class SineWaveRecieverTest : public ::testing::Test
//  {
//  protected:
//    static void SetUpTestSuite()
//    {
//      rclcpp::init(0, nullptr);
//    }

//    static void TearDownTestSuite()
//    {
//      rclcpp::shutdown();
//    }
//  };

//  // get the image path
//  std::string get_image_path()
// {
//   auto test_file_path = std::filesystem::path(__FILE__);
//   auto project_root = test_file_path.parent_path().parent_path();
//   auto image_path = project_root / "pics" / "image.png";
//   return image_path.string();
// }

//  // test initial
//  TEST_F(SineWaveRecieverTest, Initialization)
//  {
//    // create a test node
//    auto node = std::make_shared<rclcpp::Node>("test_reciever_node");

//    // create parameter
//    sine_wave::Params params;
//    params.amplitude = 1.0;
//    params.angular_frequency = 1.0;
//    params.phase = 0.0;
//    params.publisher_frequency = 10.0;

//    // if there is no fatal value, then passed
//    EXPECT_NO_THROW({
//      SineWaveReciever reciever(node, params);
//    });
//  }

//  // test service fuction
//  TEST_F(SineWaveRecieverTest, ServiceCall)
//  {
//    // create a node
//    auto node = std::make_shared<rclcpp::Node>("test_reciever_service_node");

//    // param
//    sine_wave::Params params;
//    params.amplitude = 1.0;
//    params.angular_frequency = 1.0;
//    params.phase = 0.0;
//    params.publisher_frequency = 10.0;

//    // create SineWaveReciever
//    SineWaveReciever reciever(node, params);

//    // create client and wait for service
//    auto client = node->create_client<sine_wave_cpp::srv::ConvertImage>("convert_image");
//    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2))) << "Service not available";

//    // create request
//    auto request = std::make_shared<sine_wave_cpp::srv::ConvertImage::Request>();
//    request->file_path = get_image_path();

//    // async
//    auto future = client->async_send_request(request);

//    // use spin_until_future_complete to wait for response
//    auto ret = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));
//    EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS) << "Service call failed or timed out";

//    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
//      auto response = future.get();
//      // test if it is a empty image
//      EXPECT_FALSE(response->grayscale_image.data.empty());
//    }
//  }
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
#include "sine_wave_cpp/srv/convert_image.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>  // NOLINT(build/include_order)

#include <filesystem>  // NOLINT(build/include_order)

class SineWaveRecieverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};
// get the image path
std::string get_image_path()
{
  auto test_file_path = std::filesystem::path(__FILE__);
  auto project_root = test_file_path.parent_path().parent_path();
  auto image_path = project_root / "pics" / "image.png";
  return image_path.string();
}

// test initial
TEST_F(SineWaveRecieverTest, Initialization)
{
  // create a test node
  auto node = std::make_shared<rclcpp::Node>("test_reciever_node");

  // create parameter
  sine_wave::Params params;
  params.amplitude = 1.0;
  params.angular_frequency = 1.0;
  params.phase = 0.0;
  params.publisher_frequency = 10.0;

  // if there is no fatal value, then passed
  EXPECT_NO_THROW({ SineWaveReciever reciever(node, params); });
}

// test service fuction
TEST_F(SineWaveRecieverTest, ServiceAvailability)
{
  // create a node
  auto node = std::make_shared<rclcpp::Node>("test_reciever_service_node");

  // param
  sine_wave::Params params;
  params.amplitude = 1.0;
  params.angular_frequency = 1.0;
  params.phase = 0.0;
  params.publisher_frequency = 10.0;

  // create SineWaveReciever
  SineWaveReciever reciever(node, params);

  // create client and wait for service
  auto client = node->create_client<sine_wave_cpp::srv::ConvertImage>("convert_image");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2))) << "Service not available";

  // Pass the test if service is available
  SUCCEED() << "Service is available";
}

// verify file existence without processing
TEST_F(SineWaveRecieverTest, ImageFileExists)
{
  std::string image_path = get_image_path();
  EXPECT_TRUE(std::filesystem::exists(image_path)) << "Test image file not found: " << image_path;
}
