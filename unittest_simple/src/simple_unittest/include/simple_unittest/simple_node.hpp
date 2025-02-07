#ifndef SIMPLE_NODE_HPP_INCLUDED
#define SIMPLE_NODE_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleNode : public rclcpp::Node {
public:
  SimpleNode();
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif
