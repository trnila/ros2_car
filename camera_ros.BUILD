load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_library",
)

ros2_cpp_library(
    name = "camera_ros",
    srcs = glob(
        [
            "src/*.hpp",
            "src/*.cpp",
        ],
        allow_empty = True,
    ),
    hdrs = glob(
        [
            "src/*.hpp",
        ],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
    deps = [
        "@//libcamera",
        "@image_common//:camera_info_manager",
        "@opencv",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_components",
        "@vision_opencv//:cv_bridge",
    ],
)
