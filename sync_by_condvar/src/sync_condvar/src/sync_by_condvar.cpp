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

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    this->t_ = std::thread(std::bind(&MinimalSubscriber::thread_function, this));
    rclcpp::on_shutdown(std::bind(&MinimalSubscriber::shutdown, this));
  }

private:
  void shutdown() {
    std::cout << ">> shutdown" << std::endl;
    this->finish();
    this->t_.join();
    std::cout << "<< shutdown" << std::endl;
  }

  void topic_callback(const std_msgs::msg::String &msg) {
    static int n = 1;
    if (n++ % 5 == 0) {
      put(msg.data);
      start();
    }
    RCLCPP_INFO(this->get_logger(), "==topic '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void thread_function() {
    while (1) {
      RCLCPP_INFO(this->get_logger(), "-- wait:");
      std::string v;
      {
        std::unique_lock<std::mutex> lk(mtx_);
        cond_.wait(lk, [this] { return this->buf_.has_value() || this->finish_; });
        if (this->finish_) {
          RCLCPP_INFO(this->get_logger(), "-- exit:");
          break;
        }
        v = this->buf_.value();
        this->buf_.reset();
      }
      RCLCPP_INFO(this->get_logger(), "-- work: '%s'", v.c_str());
    }
  }

  void put(const std::string &v) {
    RCLCPP_INFO(this->get_logger(), "## put: '%s'", v.c_str());
    std::lock_guard<std::mutex> lk(mtx_);
    this->buf_ = v;
  }

  void start() { cond_.notify_one(); }

  void finish() {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      this->finish_ = true;
    }
    cond_.notify_one();
  }

  std::thread t_;
  std::optional<std::string> buf_;
  std::mutex mtx_;
  std::condition_variable cond_;
  bool finish_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
