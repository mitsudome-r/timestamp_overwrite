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

#include "timestamp_overwrite/overwrite_camera_info_header.hpp"

#include <rclcpp/rclcpp.hpp>

namespace timestamp_overwrite{

OverwriteCameraInfoHeader::OverwriteCameraInfoHeader(const rclcpp::NodeOptions & options)
: Node("overwrite_camera_info_header", options)
{
  sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input", rclcpp::SensorDataQoS(),
    std::bind(&OverwriteCameraInfoHeader::onCameraInfo, this, std::placeholders::_1));
  pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/output", rclcpp::SensorDataQoS());
  frame_id_ = this->declare_parameter<std::string>("frame_id");
}

void OverwriteCameraInfoHeader::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  sensor_msgs::msg::CameraInfo fixed_msg;
  fixed_msg = *msg;
  fixed_msg.header.frame_id = frame_id_;
  pub_camera_info_->publish(fixed_msg);
}

}  // namespace timestamp_overwrite

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(timestamp_overwrite::OverwriteCameraInfoHeader)
