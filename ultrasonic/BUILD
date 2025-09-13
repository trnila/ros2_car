load("@rules_rust//rust:defs.bzl", "rust_binary")

rust_binary(
    name = "ultrasonic",
    srcs = ["ultrasonic.rs"],
    deps = [
        "//:rust_sensor_msgs",
        "@ros2_rust//:rclrs",
        "@crates//:signal-hook",
        "@crates//:gpiocdev",
    ],
    visibility = ['//visibility:public'],
)