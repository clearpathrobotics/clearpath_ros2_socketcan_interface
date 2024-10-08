#include "clearpath_ros2_socketcan_interface/socketcan_interface.hpp"

namespace clearpath_ros2_socketcan_interface
{

SocketCANInterface::SocketCANInterface(const std::string& canbus,
                                   std::shared_ptr<rclcpp::Node> & nh):
  canbus_(canbus), nh_(nh)
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

void SocketCANInterface::rxCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg)
  {
    std::lock_guard<std::mutex> lock(receive_queue_mutex_);
    can_rx_message_queue_.push(*msg);
  }
}

bool SocketCANInterface::recv(can_msgs::msg::Frame::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(receive_queue_mutex_);
  if (can_rx_message_queue_.empty())
  {
    return false;
  }
  *msg = can_rx_message_queue_.front();
  can_rx_message_queue_.pop();
  return true;
}

void SocketCANInterface::send(can_msgs::msg::Frame msg)
{
  // frame_msg_ = msg.getFrame();
  // frame_msg_.header.stamp = nh_->get_clock()->now();
  // frame_msg_.header.frame_id = "base_link";
  sleep(0.001);
  can_tx_pub_->publish(msg);
}

}  // namespace clearpath_ros2_socketcan_interface
