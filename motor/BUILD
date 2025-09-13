load("@rules_rust//rust:defs.bzl", "rust_binary")

rust_binary(
    name = "motor",
    srcs = ["motor.rs"],
    deps = [
        "//:rust_sensor_msgs",
        #"@ros2_rust//:rclrs",
        #"@crates//:signal-hook",
        "@crates//:gpiocdev",
        "@crates//:sysfs-pwm",
    ],
    visibility = ['//visibility:public'],
)