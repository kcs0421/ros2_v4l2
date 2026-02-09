// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "v4l2_camera/v4l2_camera.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp/qos.hpp>
#include <sstream>				
#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>

#include <sys/time.h>				

#include "v4l2_camera/fourcc.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace v4l2_camera
{

V4L2Camera::V4L2Camera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"v4l2_camera", options},
  canceled_{false}
{
  // Prepare camera
  auto device_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  device_descriptor.description = "Path to video device";
  device_descriptor.read_only = true;
  auto device = declare_parameter<std::string>("video_device", "/dev/video0", device_descriptor);
  
  camera_ = std::make_shared<V4l2CameraDevice>(device);

  if (!camera_->open()) {
    return;
  }
  
  // Prepare publisher
  // This should happen before registering on_set_parameters_callback,
  // else transport plugins will fail to declare their parameters
  bool use_sensor_data_qos = declare_parameter("use_sensor_data_qos", false);  
  const auto qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);
    
  if (device == "/dev/video0") {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/RGB/image_raw", qos);
  }
  else if (device == "/dev/video1") {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/IR/image_raw", qos);
  } 
  else {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(device + "/image_raw", qos);
  }

  // Read parameters and set up callback
  createParameters();

  // Start the camera
  if (!camera_->start()) {
    return;
  }

  // Start capture thread
  capture_thread_ = std::thread{
    [this]() -> void {
      while (rclcpp::ok() && !canceled_.load()) {
        RCLCPP_DEBUG(get_logger(), "Capture...");
        auto img = camera_->capture();
        if (img == nullptr) {
          // Failed capturing image, assume it is temporarily and continue a bit later
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
		  
        if (img->encoding != output_encoding_) {
          //RCLCPP_WARN_ONCE(
          //get_logger(),
            //"Image encoding not the same as requested output, performing possibly slow conversion: "
            //"%s => %s",
          //img->encoding.c_str(), output_encoding_.c_str());
          img = convert(*img);
        }
        
        img->header.frame_id = camera_frame_id_;

        image_pub_->publish(*img);
      }
    }
  };
}

V4L2Camera::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void V4L2Camera::createParameters()
{
  // Node parameters
  auto output_encoding_description = rcl_interfaces::msg::ParameterDescriptor{};
  output_encoding_description.description = "ROS image encoding to use for the output image";
  output_encoding_description.additional_constraints =
    // cv_bridge only supports converting to from color to mono of the same channels, so "mono8"
    // is not supported.
    // "mono8"
    "Currently supported: 'rgb8', 'rgba8', 'bgr8', 'bgra8', 'mono16', 'yuv422'";
  output_encoding_ = declare_parameter(
    "output_encoding", std::string{"yuv422"},
    output_encoding_description);

  auto camera_frame_id_description = rcl_interfaces::msg::ParameterDescriptor{};
  camera_frame_id_description.description = "Frame id inserted in published image";
  camera_frame_id_description.read_only = true;
  camera_frame_id_ = declare_parameter<std::string>(
    "camera_frame_id", "camera",
    camera_frame_id_description);

  // Format parameters
  // Pixel format
  auto const & image_formats = camera_->getImageFormats();
  auto pixel_format_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  pixel_format_descriptor.name = "pixel_format";
  pixel_format_descriptor.description = "Pixel format (FourCC)";
  auto pixel_format_constraints = std::ostringstream{};
  for (auto const & format : image_formats) {
    pixel_format_constraints <<
      "\"" << FourCC::toString(format.pixelFormat) << "\"" <<
      " (" << format.description << "), ";
  }
  auto str = pixel_format_constraints.str();
  str = str.substr(0, str.size() - 2);
  pixel_format_descriptor.additional_constraints = str;
  auto pixel_format =
    declare_parameter<std::string>("pixel_format", "UYVY", pixel_format_descriptor);
  requestPixelFormat(pixel_format);

  // Image size
  auto image_size = ImageSize{};
  auto image_size_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  image_size_descriptor.name = "image_size";
  image_size_descriptor.description = "Image width & height";

  // List available image sizes per format
  auto image_sizes_constraints = std::ostringstream{};
  image_sizes_constraints << "Available image sizes:";

  image_size_descriptor.additional_constraints = image_sizes_constraints.str();
  image_size = declare_parameter<ImageSize>("image_size", {1920, 1080}, image_size_descriptor);
  requestImageSize(image_size);

  // Register callback for parameter value setting
  on_set_parameters_callback_ = add_on_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const & p : parameters) {
        result.successful &= handleParameter(p);
      }
      return result;
    });
}

bool V4L2Camera::handleParameter(rclcpp::Parameter const & param)
{
  auto name = std::string{param.get_name()};
  if (param.get_name() == "output_encoding") {
    output_encoding_ = param.as_string();
    return true;
  } else if (param.get_name() == "pixel_format") {
    camera_->stop();
    auto success = requestPixelFormat(param.as_string());
    camera_->start();
    return success;
  } else if (param.get_name() == "image_size") {
    camera_->stop();
    auto success = requestImageSize(param.as_integer_array());
    camera_->start();
    return success;
  } else{
  
  }

  return false;
}

bool V4L2Camera::requestPixelFormat(std::string const & fourcc)
{
  if (fourcc.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Invalid pixel format size: must be a 4 character code (FOURCC).");
    return false;
  }

  auto code = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given pixel format
  if (dataFormat.pixelFormat == code) {
    return true;
  }

  dataFormat.pixelFormat = code;
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestImageSize(std::vector<int64_t> const & size)
{
  if (size.size() != 2) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid image size; expected dimensions: 2, actual: %ld",
      size.size());
    return false;
  }

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given size
  if (dataFormat.width == size[0] && dataFormat.height == size[1]) {
    return true;
  }

  dataFormat.width = size[0];
  dataFormat.height = size[1];
  return camera_->requestDataFormat(dataFormat);
}



sensor_msgs::msg::Image::UniquePtr V4L2Camera::convert(sensor_msgs::msg::Image const & img) const
{
  // Change ros image to cv::Mat
  auto tracked_object = std::shared_ptr<const void>{};
  auto cv_img = cv_bridge::toCvShare(img, tracked_object);
  
  // Convert outpur_encoding
  auto convert_img = cv_bridge::cvtColor(cv_img, output_encoding_);
  
  // Create image object
  auto out_img = std::make_unique<sensor_msgs::msg::Image>();
  
  // Change cv::Mat to ros image
  convert_img->toImageMsg(*out_img);
  
  return out_img;

}

}  // namespace v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
