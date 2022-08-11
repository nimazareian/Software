package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sml_to_plant_uml",
    srcs = [
        "sml_to_plant_uml.cpp",
    ],
    hdrs = [
        "sml_to_plant_uml.h",
    ],
    deps = [
        "@sml",
        "//software/ai:ai",
    ],
)

cc_test(
    name = "sml_to_plant_uml_test",
    srcs = [
        "sml_to_plant_uml_test.cpp",
    ],
    deps = [
        ":sml_to_plant_uml",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai:ai",
    ],
)