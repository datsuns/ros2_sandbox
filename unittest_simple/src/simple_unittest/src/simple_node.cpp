
#include "simple_unittest/simple_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

SimpleNode::SimpleNode() : Node("simple_node") {
  publisher_ = this->create_publisher<std_msgs::msg::String>("output_topic", 10);
  subscription_ =
      this->create_subscription<std_msgs::msg::String>("input_topic", 10, std::bind(&SimpleNode::topic_callback, this, std::placeholders::_1));
}

void SimpleNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  auto message = std::make_shared<std_msgs::msg::String>();
  message->data = "Processed: " + msg->data;
  publisher_->publish(*message);
}

#if 0
// メイン関数
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif
