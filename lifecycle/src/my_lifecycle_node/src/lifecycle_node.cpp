
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::placeholders;

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node") {
        RCLCPP_INFO(get_logger(), "Started...");
    }

protected:
    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring...");
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating...");
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto lifecycle_node = std::make_shared<MyLifecycleNode>();
    rclcpp::spin(lifecycle_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
