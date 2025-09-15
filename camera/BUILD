load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")

ros2_cpp_binary(
    name = "camera",
    srcs = ["main.cpp"],
    data = [
        "@camera_ros",
    ],
    deps = [
        "@ros2_class_loader//:class_loader",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
    ],
)
