load("@rules_rust//rust:defs.bzl", "rust_binary")

rust_binary(
    name = "ultrasonic",
    srcs = ["ultrasonic.rs"],
    visibility = ["//visibility:public"],
    deps = [
        "//:rust_sensor_msgs",
        "@crates//:gpiocdev",
        "@crates//:signal-hook",
        "@ros2_rust//:rclrs",
    ],
)
