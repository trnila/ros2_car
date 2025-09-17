load(
    "@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl",
    "ros2_cpp_library",
)

ros2_cpp_library(
    name = "cv_bridge",
    srcs = [
        "cv_bridge/src/cv_bridge.cpp",
        "cv_bridge/src/rgb_colors.cpp",
        ":cv_bridge_export",
    ],
    hdrs = glob([
        "cv_bridge/include/**",
    ]),
    includes = ["cv_bridge/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@boost.endian",
        "@opencv",
        "@ros2_common_interfaces//:cpp_sensor_msgs",
    ],
)

genrule(
    name = "cv_bridge_export",
    outs = ["cv_bridge/include/cv_bridge/cv_bridge_export.h"],
    cmd = """
cat > $@ <<'EOF'
#ifndef CV_BRIDGE_EXPORT_H
#define CV_BRIDGE_EXPORT_H

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CV_BRIDGE_EXPORT __attribute__ ((dllexport))
    #define CV_BRIDGE_NO_EXPORT __attribute__ ((dllimport))
  #else
    #define CV_BRIDGE_EXPORT __declspec(dllexport)
    #define CV_BRIDGE_NO_EXPORT __declspec(dllimport)
  #endif
#else
  #define CV_BRIDGE_EXPORT __attribute__ ((visibility ("default")))
  #define CV_BRIDGE_NO_EXPORT
#endif

#endif /* CV_BRIDGE_EXPORT_H */
EOF
""",
)
