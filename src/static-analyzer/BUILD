package(default_visibility = ["//visibility:public"])

load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")

py_binary(
    name = "generate_compile_command",
    srcs = [
        "generate_compile_command.py",
    ],
    deps = [
        "//proto:tbots_py_proto",
    ],
)

action_listener(
    name = "generate_compile_commands_listener",
    extra_actions = [":generate_compile_commands_action"],
    mnemonics = [
        "CppCompile",
    ],
    visibility = ["//visibility:public"],
)

extra_action(
    name = "generate_compile_commands_action",
    cmd = "$(location :generate_compile_command) $(EXTRA_ACTION_FILE)" +
          " $(output $(ACTION_ID)_compile_command)",
    out_templates = [
        "$(ACTION_ID)_compile_command",
    ],
    tools = [
        ":generate_compile_command",
    ],
)

#py_proto_library(
#  name = "extra_actions_py_proto",
#  srcs = [
#      "extra_actions_base.proto",
#  ],
#  visibility = ["//visibility:public"],
##     deps = [
##         "@nanopb//:nanopb_options_py_proto",
##     ],
#)
