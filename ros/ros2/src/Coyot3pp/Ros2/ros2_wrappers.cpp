#include <milla_connect_tools/ros2_node_interface/ros2_wrappers.h>




namespace milla{
namespace connapp{
namespace ros2{

  rclcpp::Time get_ros_time(){
    return rclcpp::Clock{RCL_ROS_TIME}.now();
  }

  std_msgs::msg::Header generate_header_packet(int32_t seq){
    std_msgs::msg::Header h;
    h.stamp = rclcpp::Time(static_cast<uint32_t>(milla::connapp::getCurrentTimestamp()));
    if(seq){
      h.frame_id = std::to_string(seq);
    }
    return h;
  }
  std_msgs::msg::Header
  generate_header_packet(const std::string& frameId){
    std_msgs::msg::Header h;
    h.stamp = rclcpp::Time(static_cast<uint32_t>(milla::connapp::getCurrentTimestamp()));
    h.frame_id = frameId;
    return h;
  }


}
}
}
