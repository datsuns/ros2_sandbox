#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PclSampleNode : public rclcpp::Node
{
public:
  PclSampleNode() : Node("pcl_sample_node")
  {
    RCLCPP_INFO(this->get_logger(), "PCL Sample Node (Humble) started.");
    
    // Create a dummy PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 1;
    cloud.height = 1;
    cloud.points.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));

    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    
    RCLCPP_INFO(this->get_logger(), "Converted PCL to ROS msg: width=%d", ros_msg.width);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclSampleNode>());
  rclcpp::shutdown();
  return 0;
}
