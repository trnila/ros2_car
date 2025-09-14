load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_library")

rust_library(
    name = "car_controller",
    srcs = ["car_controller.rs"],
    deps = [
        "@crates//:gpiocdev",
        "@crates//:sysfs-pwm",
    ],
)

rust_binary(
    name = "cli",
    srcs = ["cli.rs"],
    deps = [
        ":car_controller",
        "@crates//:clap",
        "@crates//:clap-repl",
    ],
)

rust_binary(
    name = "motor",
    srcs = ["motor.rs"],
    visibility = ["//visibility:public"],
    deps = [
        "//:rust_sensor_msgs",
        #"@ros2_rust//:rclrs",
        #"@crates//:signal-hook",
        "@crates//:gpiocdev",
        "@crates//:sysfs-pwm",
    ],
)
