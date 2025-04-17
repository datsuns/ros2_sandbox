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
#include <regex>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    run_cmd("ros2 topic list");
    auto nodes = run_cmd("ros2 node list");
    show_node_details(nodes);
  }

  std::string run_cmd(const std::string &cmd) {
    auto raw = run(cmd);
    auto log = std::string{"cmd ["} + cmd + "]\n" + raw;
    RCLCPP_INFO(this->get_logger(), log.c_str());
    return raw;
  }

  std::string run(const std::string &cmd) {
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
      RCLCPP_ERROR(this->get_logger(), "popen failed!");
      return "";
    }

    char buffer[128];
    std::ostringstream log;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      log << buffer;
    }
    pclose(pipe);
    return log.str();
  }

  void show_node_details(const std::string &node_list_log) {
    std::string log{"[show node details]\n"};
    for (auto line : split_lines(node_list_log)) {
      auto cmd = std::string{"ros2 node info "} + line;
      auto one_node = run(cmd);
      log += one_node + "\n";
    }
    RCLCPP_INFO(this->get_logger(), log.c_str());
  }

  std::vector<std::string> split_lines(const std::string &input) {
    std::regex re(R"(\r\n|\n|\r)");
    std::sregex_token_iterator it(input.begin(), input.end(), re, -1);
    std::sregex_token_iterator end;
    std::vector<std::string> lines(it, end);
    return lines;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
