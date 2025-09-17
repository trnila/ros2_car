load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_binary",
)

ros2_cpp_binary(
    name = "camera_ros",
    srcs = glob(
        [
            "src/*.hpp",
            "src/*.cpp",
        ],
    ),
    copts = [
        "-I/usr/include/libcamera",
    ],
    linkopts = [
        "-lcamera",
        "-lcamera-base",
    ],
    # TODO: enable when camera is built from source
    target_compatible_with = [
        "@platforms//cpu:aarch64",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@opencv",
        "@ros2_image_common//:camera_info_manager",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
        "@vision_opencv//:cv_bridge",
    ],
)
