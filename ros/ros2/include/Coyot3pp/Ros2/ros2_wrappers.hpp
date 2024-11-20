#pragma once


#include <Coyot3pp/Cor3/Coyot3.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

namespace milla{
namespace connapp{
namespace ros2{

  rclcpp::Time get_ros_time();
  std_msgs::msg::Header generate_header_packet(int32_t seq = 0);
  std_msgs::msg::Header generate_header_packet(const std::string& frameId);



}
}
}