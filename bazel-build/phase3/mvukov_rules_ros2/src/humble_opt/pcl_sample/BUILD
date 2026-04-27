load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")

ros2_cpp_binary(
    name = "pcl_sample_node",
    srcs = ["src/pcl_sample_node.cpp"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_pcl_conversions//:pcl_conversions",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
        "@pcl//:pcl",
        "@eigen//:eigen",
    ],
)
