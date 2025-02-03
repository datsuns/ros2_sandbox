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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    this->n = 0;
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions options;
    options.callback_group = client_cb_group_;
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1), options);

    timer_ = this->create_wall_timer(1s, std::bind(&MinimalSubscriber::timer_callback, this), client_cb_group_);

    t = std::thread(&MinimalSubscriber::thread_function, this);

    rclcpp::on_shutdown(std::bind(&MinimalSubscriber::cleanup, this));
  }

private:
  void cleanup() {
    std::cout << ">> cleanup" << std::endl;
    this->finish = true;
    t.join();
    std::cout << "<< cleanup" << std::endl;
  }

  void thread_function() {
    while (!this->finish) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      std::cout << "thread" << std::endl;
    }
    std::cout << "finish" << std::endl;
  }

  void topic_callback(const std_msgs::msg::String &msg) const { RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str()); }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer");
    std::thread([this]() { this->long_task(this->n++); }).detach();
  }

  void long_task(int n) {
    RCLCPP_INFO(this->get_logger(), ">> long_task %d", n);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // std::future<void> result =
    //     std::async(std::launch::async, &MinimalSubscriber::long_task, this);

    // std::async(std::launch::async, &MinimalSubscriber::long_task, this);
    RCLCPP_INFO(this->get_logger(), "<< long_task %d", n);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  std::thread t;
  bool finish = false;
  int n;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
#if 0
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
#else
  rclcpp::spin(node);
#endif
  rclcpp::shutdown();
  return 0;
}
