load("@rules_foreign_cc//foreign_cc:meson.bzl", "meson")

filegroup(
    name = "src",
    srcs = glob(["**"]),
)

meson(
    name = "libcamera",
    lib_source = ":src",
    options = {
        "pipelines": "rpi/vc4,rpi/pisp",
        "ipas": "rpi/vc4,rpi/pisp",
        "v4l2": "disabled",
        "gstreamer": "disabled",
        "test": "false",
        "lc-compliance": "disabled",
        "cam": "disabled",
        "qcam": "disabled",
        "documentation": "disabled",
        "pycamera": "disabled",
    },
    out_include_dir = "include/libcamera",
    out_lib_dir = select({
        "@platforms//cpu:x86_64": "lib/x86_64-linux-gnu",
        "@platforms//cpu:arm64": "lib",
    }),
    out_shared_libs = [
        "libcamera-base.so",
        "libcamera.so",
        "libpisp.so",
        "libcamera/ipa/ipa_rpi_pisp.so",
        "libcamera/ipa/ipa_rpi_vc4.so",
    ],
    tags = [
        "requires-network",
    ],
    visibility = ["//visibility:public"],
)
