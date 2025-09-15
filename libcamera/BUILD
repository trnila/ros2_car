load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "libcamera",
    srcs = glob(["lib/*"]),
    hdrs = glob(["libcamera/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
