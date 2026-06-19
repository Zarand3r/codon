# L0 — 128-bit hashing (Hash128 + digest_xxh128). Header-only.

# cc_library / cc_test are native (autoloaded) rules in Bazel 8.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hash",
    hdrs = [
        "Hash128.h",
        "xxh.h",
    ],
    strip_include_prefix = "/vehicle",
    deps = ["//vehicle/src/bullwinkle/all:core"],  # drone_types.h
)

cc_test(
    name = "hash_test",
    srcs = ["hash_test.cc"],
    copts = ["-std=c++17"],
    deps = [":hash"],
)
