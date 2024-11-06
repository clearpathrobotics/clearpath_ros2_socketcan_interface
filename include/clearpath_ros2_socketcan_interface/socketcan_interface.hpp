#ifndef CLEARPATH_ROS2_SOCKETCAN_INTERFACE__SOCKETCAN_INTERFACE_HPP_
#define CLEARPATH_ROS2_SOCKETCAN_INTERFACE__SOCKETCAN_INTERFACE_HPP_


#include <queue>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "clearpath_ros2_socketcan_interface/visibility_control.h"

namespace clearpath_ros2_socketcan_interface
{

class SocketCANInterface
{
public:
  SocketCANInterface(const std::string & canbus, std::shared_ptr<rclcpp::Node> & nh);

  bool recv(can_msgs::msg::Frame::SharedPtr msg);
  void send(can_msgs::msg::Frame msg);

private:
  std::string canbus_;  // CANBUS interface
  std::shared_ptr<rclcpp::Node> nh_;

  std::queue<can_msgs::msg::Frame> can_rx_message_queue_;
  std::mutex receive_queue_mutex_;

  can_msgs::msg::Frame frame_msg_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;

  void rxCallback(const can_msgs::msg::Frame::SharedPtr msg);
};

}
  // namespace clearpath_ros2_socketcan_interface

#endif  // CLEARPATH_ROS2_SOCKETCAN_INTERFACE__SOCKETCAN_INTERFACE_HPP_
