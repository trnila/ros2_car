load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_library",
)

ros2_cpp_library(
    name = "camera_info_manager",
    srcs = glob(
        [
            "camera_info_manager/*.hpp",
            "camera_info_manager/*.cpp",
        ],
        allow_empty = True,
    ),
    hdrs =
        [
            "camera_info_manager/include/camera_info_manager/camera_info_manager.hpp",
            "camera_info_manager/include/camera_info_manager/visibility_control.h",
        ],
    includes = ["camera_info_manager/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_common_interfaces//:cpp_sensor_msgs",
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rclcpp//:rclcpp_lifecycle",
    ],
)
