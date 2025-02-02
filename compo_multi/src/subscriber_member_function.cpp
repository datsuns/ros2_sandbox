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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node {
public:
  // MinimalSubscriber() : Node("minimal_subscriber") {
  //   subscription_ = this->create_subscription<std_msgs::msg::String>(
  //       "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this,
  //       _1));
  // }

  MinimalSubscriber(const rclcpp::NodeOptions &options)
      : Node("minimal_subscriber", options) {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalSubscriber::timer_callback, this));
    std::thread t([]() {
      while (1) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "thread" << std::endl;
      }
    });
  }

private:
  void topic_callback(const std_msgs::msg::String &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  void timer_callback() {
    static int times = 100;
    RCLCPP_INFO(this->get_logger(), "Timer");
    if (times < 10) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    times++;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MinimalSubscriber)
