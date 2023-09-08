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

#include "timestamp_overwrite/overwrite_image_header.hpp"

#include <rclcpp/rclcpp.hpp>

namespace timestamp_overwrite{

OverwriteImageHeader::OverwriteImageHeader(const rclcpp::NodeOptions & options)
: Node("overwrite_image_header", options)
{
  sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
    "~/input", rclcpp::SensorDataQoS(),
    std::bind(&OverwriteImageHeader::onImage, this, std::placeholders::_1));
  pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("~/output", rclcpp::SensorDataQoS());
  frame_id_ = this->declare_parameter<std::string>("frame_id");
}

void OverwriteImageHeader::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  sensor_msgs::msg::Image fixed_msg;
  fixed_msg = *msg;
  fixed_msg.header.frame_id = frame_id_;
  pub_image_->publish(fixed_msg);
}

}  // namespace timestamp_overwrite

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(timestamp_overwrite::OverwriteImageHeader)
