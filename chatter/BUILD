load("@com_github_mvukov_rules_ros2//ros2:bag.bzl", "ros2_bag")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")
load("@com_github_mvukov_rules_ros2//ros2:launch.bzl", "ros2_launch")
load("@com_github_mvukov_rules_ros2//ros2:rust_interfaces.bzl", "rust_ros2_interface_library")
load("@com_github_mvukov_rules_ros2//ros2:test.bzl", "ros2_test")
load("@com_github_mvukov_rules_ros2//ros2:topic.bzl", "ros2_topic")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_rust//rust:defs.bzl", "rust_binary")

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

