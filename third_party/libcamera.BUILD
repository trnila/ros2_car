load("@rules_foreign_cc//foreign_cc:meson.bzl", "meson")

filegroup(
    name = "src",
    srcs = glob(["**"]),
)

meson(
    name = "libcamera",
    lib_source = ":src",
    tags = [
        "requires-network",
    ],
)
