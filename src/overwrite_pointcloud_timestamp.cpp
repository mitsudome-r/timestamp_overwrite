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

#include "timestamp_overwrite/overwrite_pointcloud_timestamp.hpp"

#include <rclcpp/rclcpp.hpp>

namespace timestamp_overwrite{

OverwritePointCloudTimeStamp::OverwritePointCloudTimeStamp(const rclcpp::NodeOptions & options)
: Node("overwrite_pointcloud_timestamp", options)
{
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input", rclcpp::SensorDataQoS(),
    std::bind(&OverwritePointCloudTimeStamp::onPointCloud, this, std::placeholders::_1));
  pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/output", rclcpp::SensorDataQoS());
}

void OverwritePointCloudTimeStamp::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 fixed_timestamp_msg;
  fixed_timestamp_msg = *msg;
  fixed_timestamp_msg.header.stamp = this->now();
  pub_pointcloud_->publish(fixed_timestamp_msg);
}

}  // namespace timestamp_overwrite

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(timestamp_overwrite::OverwritePointCloudTimeStamp)
