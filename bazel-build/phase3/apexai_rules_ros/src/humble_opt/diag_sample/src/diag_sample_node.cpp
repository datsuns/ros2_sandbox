#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

class DiagSampleNode : public rclcpp::Node
{
public:
  DiagSampleNode() : Node("diag_sample_node"), updater_(this)
  {
    updater_.setHardwareID("sample_hardware_humble");
    updater_.add("sample_task", this, &DiagSampleNode::check_task);
    
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { updater_.force_update(); });
  }

private:
  void check_task(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Task is running fine on Humble");
    stat.add("key", "value_humble");
    RCLCPP_INFO(this->get_logger(), "Diagnostic updated.");
  }

  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagSampleNode>());
  rclcpp::shutdown();
  return 0;
}
