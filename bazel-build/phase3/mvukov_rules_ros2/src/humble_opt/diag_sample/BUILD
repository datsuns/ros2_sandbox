load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")

ros2_cpp_binary(
    name = "diag_sample_node",
    srcs = ["src/diag_sample_node.cpp"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_diagnostics//:cpp_diagnostic_updater",
    ],
)
