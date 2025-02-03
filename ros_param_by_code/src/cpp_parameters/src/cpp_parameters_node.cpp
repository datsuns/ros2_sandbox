#include <chrono>
#include <functional>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node {

public:
  MinimalParam() : Node("minimal_param_node") {
    this->declare_parameter("my_parameter", "world");
    this->my_param = this->get_parameter("my_parameter").as_string();
    std::cout << "param is " << this->my_param << std::endl;
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalParam::timer_callback, this));
  }

#if 0
   // だめ。Node()の継承の定義が重複して二重にノードを作ることになる
   MinimalParam(const rclcpp::NodeOptions &opt)
       : Node("minimal_param_node", opt) {
     MinimalParam();
   }
#else
  MinimalParam(const rclcpp::NodeOptions &opt) : MinimalParam() {
    for (auto param : opt.parameter_overrides()) {
      if (param.get_name() == "my_parameter") {
        this->my_param = param.as_string();
      }
    }
  }
#endif

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::string my_param;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
#if 0
  rclcpp::spin(std::make_shared<MinimalParam>());
#else
  rclcpp::NodeOptions opt = rclcpp::NodeOptions();
  opt.parameter_overrides().push_back(
      rclcpp::Parameter("my_parameter", "CUSTOM VALUE2"));
  rclcpp::spin(std::make_shared<MinimalParam>(opt));
#endif
  rclcpp::shutdown();
  return 0;
}
