// Copyright 2023 The Autoware Contributors
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

#ifndef OVERWRITE_IMAGE_HEADER_HPP_
#define OVERWRITE_IMAGE_HEADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace timestamp_overwrite{

class OverwriteImageHeader : public rclcpp::Node
{
public:
  explicit OverwriteImageHeader(const rclcpp::NodeOptions & options): Node("overwrite_header", options){
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_input", rclcpp::SensorDataQoS(),
        std::bind(&OverwriteImageHeader::onImageMessage, this, std::placeholders::_1));
    compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "compressed_image_input", rclcpp::SensorDataQoS(),
        std::bind(&OverwriteImageHeader::onCompressedImageMessage, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info_input", rclcpp::SensorDataQoS(),
        std::bind(&OverwriteImageHeader::onCameraInfoMessage, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_output", rclcpp::SensorDataQoS());
    compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_image_output", rclcpp::SensorDataQoS());
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_output", rclcpp::SensorDataQoS());
    overwrite_frame_id_ = this->declare_parameter<bool>("overwrite_frame_id");
    if (overwrite_frame_id_){
      frame_id_ = this->declare_parameter<std::string>("frame_id");
    }
  }

private:
  void onImageMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg){
      sensor_msgs::msg::Image fixed_msg;
      fixed_msg = *msg;
      if (overwrite_frame_id_){
        fixed_msg.header.frame_id = frame_id_;
      }
      fixed_msg.header.stamp = this->now();
      camera_info_.header.stamp = fixed_msg.header.stamp;

      image_pub_->publish(fixed_msg);
      camera_info_pub_->publish(camera_info_);
  }
  void onCameraInfoMessage(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
      camera_info_ = *msg;
      if (overwrite_frame_id_){
        camera_info_.header.frame_id = frame_id_;
      }
  }
  void onCompressedImageMessage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){
      compressed_image_ = *msg;
      if (overwrite_frame_id_){
        compressed_image_.header.frame_id = frame_id_;
      }
      const auto current_time = this->now();
      compressed_image_.header.stamp = current_time;
      camera_info_.header.stamp = current_time;
      compressed_image_pub_->publish(compressed_image_);
      camera_info_pub_->publish(camera_info_);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
  sensor_msgs::msg::CameraInfo camera_info_;
  sensor_msgs::msg::CompressedImage compressed_image_;
  bool overwrite_frame_id_ = false;
  std::string frame_id_;

};

}  // namespace timestamp_overwrite

#endif  // OVERWRITE_IMAGE_HEADER_HPP_
