load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")

ros2_cpp_binary(
    name = "talker",
    srcs = ["talker.cc"],
    deps = [
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_rclcpp//:rclcpp",
    ],
)

ros2_cpp_binary(
    name = "listener",
    srcs = ["listener.cc"],
    deps = [
        "@ros2_common_interfaces//:cpp_std_msgs",
        "@ros2_rclcpp//:rclcpp",
    ],
)

platform(
    name = "aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
)
