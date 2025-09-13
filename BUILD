load("@aarch64_none_linux_gnu//toolchain:toolchain.bzl", "aarch64_none_linux_gnu_toolchain")

platform(
    name = "aarch64",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
)

aarch64_none_linux_gnu_toolchain(
    name = "aarch64_none_linux_gnu",
    copts = [
        "-fPIC",
    ],
    linkopts = [
        "-lc",
        "-lstdc++",
        "-lm",
    ],
)
