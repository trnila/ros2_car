load("@com_github_mvukov_rules_ros2//ros2:topic.bzl", "ros2_topic")
load("@com_github_mvukov_rules_ros2//ros2:rust_interfaces.bzl", "rust_ros2_interface_library")
load("@com_github_mvukov_rules_ros2//ros2:launch.bzl", "ros2_launch")


platform(
    name = "aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
)

rust_ros2_interface_library(
    name = "rust_std_msgs",
    deps = ["@ros2_common_interfaces//:std_msgs"],
    visibility = ["//visibility:public"],
)

rust_ros2_interface_library(
    name = "rust_sensor_msgs",
    deps = ["@ros2_common_interfaces//:sensor_msgs"],
    visibility = ["//visibility:public"],
)

ros2_topic(
    name = "topic",
    deps = [
        "@ros2_common_interfaces//:py_std_msgs",
        "@ros2_common_interfaces//:py_sensor_msgs",
    ],
)

ros2_launch(
    name = "car",
    launch_file = "car.launch.py",
    nodes = [
        "//ultrasonic",
    ],
)