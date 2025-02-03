// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
namespace chrono = std::chrono;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0),
        prev(chrono::system_clock::now()) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    auto diff = chrono::system_clock::now() - this->prev;
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    RCLCPP_INFO(
        this->get_logger(), "diff: '%ld'",
        std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
    publisher_->publish(message);
    std::this_thread::sleep_for(chrono::seconds(2));
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  chrono::system_clock::time_point prev;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
#if 0
  rclcpp::spin(std::make_shared<MinimalPublisher>());
#else
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<MinimalPublisher>();
  exec.add_node(node);
  exec.spin();
#endif
  rclcpp::shutdown();
  return 0;
}
