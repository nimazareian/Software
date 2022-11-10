package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "proto_configuration_widget",
    srcs = ["proto_configuration_widget.py"],
)

py_library(
    name = "proto_plotter",
    srcs = ["proto_plotter.py"],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
        requirement("vispy"),
    ],
)

py_library(
    name = "common_widgets",
    srcs = ["common_widgets.py"],
)
