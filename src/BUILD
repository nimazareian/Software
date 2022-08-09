load("@rules_python//python:defs.bzl", "py_runtime")
load("@rules_python//python:defs.bzl", "py_runtime_pair")

py_runtime(
    name = "python3_runtime",
    interpreter_path = "/opt/tbotspython/bin/python3",
    python_version = "PY3",
    visibility = ["//visibility:public"],
)

py_runtime_pair(
    name = "py_runtime_pair",
    py2_runtime = None,
    py3_runtime = ":python3_runtime",
)

toolchain(
    name = "py_toolchain",
    toolchain = ":py_runtime_pair",
    toolchain_type = "@bazel_tools//tools/python:toolchain_type",
)

load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

# Generates compile_commands.json
refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = [
        "//software/...",
        "//proto/...",
        "//extlibs/...",
        "//shared/...",
    ],
)

filegroup(
    name = "clang_tidy_config_default",
    data = [
        ".clang-tidy",
#        "//:clang_tidy_config",
    ],
)

label_flag(
    name = "clang_tidy_config",
    build_setting_default = ":clang_tidy_config_default",
    visibility = ["//visibility:public"],
)
