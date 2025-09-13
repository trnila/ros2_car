load("@com_github_mvukov_rules_ros2//ros2:rust_interfaces.bzl", "rust_ros2_interface_library")
load("@rules_rust//rust:defs.bzl", "rust_binary")

rust_binary(
    name = "rust_talker",
    srcs = ["ultrasonic.rs"],
    deps = [
        ":rust_std_msgs",
        "@ros2_rust//:rclrs",
        "@crates//:signal-hook",
        "@crates//:gpiocdev",
    ],
)

rust_ros2_interface_library(
    name = "rust_std_msgs",
    deps = ["@ros2_common_interfaces//:std_msgs"],
)
