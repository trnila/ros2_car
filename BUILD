load("@com_github_mvukov_rules_ros2//ros2:topic.bzl", "ros2_topic")


platform(
    name = "aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
)

ros2_topic(
    name = "topic",
    deps = [
        "@ros2_common_interfaces//:py_std_msgs",
        "@ros2_common_interfaces//:py_sensor_msgs",
    ],
)