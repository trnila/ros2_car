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
    tags = [
        "requires-network",
    ],
)
