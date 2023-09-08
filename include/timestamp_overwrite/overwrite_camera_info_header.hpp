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

#ifndef OVERWRITE_CAMERA_INFO_HEADER_HPP_
#define OVERWRITE_CAMERA_INFO_HEADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace timestamp_overwrite{

class OverwriteCameraInfoHeader : public rclcpp::Node
{
public:
  explicit OverwriteCameraInfoHeader(const rclcpp::NodeOptions & options);

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
  std::string frame_id_;

};

}  // namespace timestamp_overwrite

#endif  // OVERWRITE_CAMERA_INFO_HEADER_HPP_
