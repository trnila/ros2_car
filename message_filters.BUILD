load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_binary",
    "ros2_cpp_library",
)


ros2_cpp_library(
    name = "message_filters",
    hdrs = glob([
        "include/**/*.hpp",
        "include/**/*.h",
    ]),
    visibility = ['//visibility:public'],
    includes = ['include'],
)