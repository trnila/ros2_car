load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_library",
)
load(
    "@com_github_mvukov_rules_ros2//ros2:interfaces.bzl",
    "cpp_ros2_interface_library",
    "ros2_interface_library",
)

DEPS = [
    "@ros2_common_interfaces//:cpp_std_msgs",
    "@ros2_rclcpp//:rclcpp",
    "@ros2_rclcpp//:rclcpp_lifecycle",
]

ros2_interface_library(
    name = "bond",
    srcs = glob(["bond/msg/*.msg"]),
    visibility = ["//visibility:public"],
)

cpp_ros2_interface_library(
    name = "cpp_bond",
    visibility = ["//visibility:public"],
    deps = [
        ":bond",
    ],
)

ros2_cpp_library(
    name = "bondcpp",
    srcs = glob(
        [
            "bondcpp/**/*.hpp",
            "bondcpp/**/*.cpp",
        ],
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "bondcpp/**/*.hpp",
        ],
    ),
    includes = ["bondcpp/include"],
    visibility = ["//visibility:public"],
    deps = DEPS + [
        ":cpp_bond",
        ":smclib",
    ],
)

ros2_cpp_library(
    name = "smclib",
    srcs = glob(
        [
            "smclib/**/*.hpp",
        ],
        exclude = [
            "**/test/**",
        ],
    ),
    hdrs = glob(
        [
            "smclib/**/*.hpp",
        ],
    ),
    includes = ["smclib/include"],
    visibility = ["//visibility:public"],
    deps = DEPS + [
    ],
)
