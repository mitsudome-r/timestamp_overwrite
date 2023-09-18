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

#ifndef OVERWRITE_HEADER_HPP_
#define OVERWRITE_HEADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace timestamp_overwrite{

template <typename T>
class OverwriteHeader : public rclcpp::Node
{
public:
  explicit OverwriteHeader(const rclcpp::NodeOptions & options): Node("overwrite_header", options){
    sub_ = this->create_subscription<T>(
        "input", rclcpp::SensorDataQoS(),
        std::bind(&OverwriteHeader::onMessage, this, std::placeholders::_1));
    pub_ = this->create_publisher<T>("output", rclcpp::SensorDataQoS());
    overwrite_frame_id_ = this->declare_parameter<bool>("overwrite_frame_id");
    if (overwrite_frame_id_){
      frame_id_ = this->declare_parameter<std::string>("frame_id");
    }
  }

private:
  void onMessage(const typename T::ConstSharedPtr msg){
      T fixed_msg;
      fixed_msg = *msg;
      if (overwrite_frame_id_){
        fixed_msg.header.frame_id = frame_id_;
      }
      fixed_msg.header.stamp = this->now();
      pub_->publish(fixed_msg);
  }

  typename rclcpp::Subscription<T>::SharedPtr sub_;
  typename rclcpp::Publisher<T>::SharedPtr pub_;
  bool overwrite_frame_id_ = false;
  std::string frame_id_;

};

}  // namespace timestamp_overwrite

#endif  // OVERWRITE_HEADER_HPP_
