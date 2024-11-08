/**
Software License Agreement (BSD)

\authors   Roni Kreinin <rkreinin@clearpathrobotics.com>
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"

namespace clearpath_ros2_socketcan_interface
{

SocketCANInterface::SocketCANInterface(
  const std::string & canbus,
  std::shared_ptr<rclcpp::Node> & nh
)
: canbus_(canbus), nh_(nh)
{
  // Rx Subscriber
  can_rx_sub_ = nh->create_subscription<can_msgs::msg::Frame>(
    canbus_ + "/rx",
    rclcpp::SensorDataQoS(),
    std::bind(&SocketCANInterface::rxCallback, this, std::placeholders::_1));

  // Tx Publisher
  can_tx_pub_ = nh->create_publisher<can_msgs::msg::Frame>(
    canbus_ + "/tx",
    rclcpp::SystemDefaultsQoS());
}

SocketCANInterface::SocketCANInterface(
    const std::string& canbus,
    std::shared_ptr<rclcpp::Node> & nh, 
    std::function<void(const can_msgs::msg::Frame::SharedPtr msg)> cb
)
: canbus_(canbus), nh_(nh)
{
  // Rx Subscriber
  can_rx_sub_ = nh->create_subscription<can_msgs::msg::Frame>(
    canbus_ + "/rx",
    rclcpp::SensorDataQoS(),
    cb);

  // Tx Publisher
  can_tx_pub_ = nh->create_publisher<can_msgs::msg::Frame>(
    canbus_ + "/tx",
    rclcpp::SystemDefaultsQoS());
}

void SocketCANInterface::rxCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg) {
    std::lock_guard<std::mutex> lock(receive_queue_mutex_);
    can_rx_message_queue_.push(*msg);
  }
}

bool SocketCANInterface::recv(can_msgs::msg::Frame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(receive_queue_mutex_);
  if (can_rx_message_queue_.empty()) {
    return false;
  }
  *msg = can_rx_message_queue_.front();
  can_rx_message_queue_.pop();
  return true;
}

void SocketCANInterface::send(can_msgs::msg::Frame msg)
{
  frame_msg_ = msg.getFrame();
  frame_msg_.header.stamp = nh_->get_clock()->now();
  can_tx_pub_->publish(msg);
}

}  // namespace clearpath_ros2_socketcan_interface
