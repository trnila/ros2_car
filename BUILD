load("@com_github_mvukov_rules_ros2//ros2:launch.bzl", "ros2_launch")
load("@com_github_mvukov_rules_ros2//ros2:rust_interfaces.bzl", "rust_ros2_interface_library")
load("@com_github_mvukov_rules_ros2//ros2:topic.bzl", "ros2_topic")

platform(
    name = "aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
)

rust_ros2_interface_library(
    name = "rust_std_msgs",
    visibility = ["//visibility:public"],
    deps = ["@ros2_common_interfaces//:std_msgs"],
)

rust_ros2_interface_library(
    name = "rust_sensor_msgs",
    visibility = ["//visibility:public"],
    deps = ["@ros2_common_interfaces//:sensor_msgs"],
)

ros2_topic(
    name = "topic",
    deps = [
        "@ros2_common_interfaces//:py_sensor_msgs",
        "@ros2_common_interfaces//:py_std_msgs",
    ],
)

ros2_launch(
    name = "car.launch",
    launch_file = "car.launch.py",
    nodes = [
        "//ultrasonic",
        "@camera_ros",
    ],
)
