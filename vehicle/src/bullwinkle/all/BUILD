# L0 — core primitives (data-oriented-design layer). Header-only; no external deps.
# `strip_include_prefix = "/vehicle"` exposes headers as `src/bullwinkle/all/...`,
# matching the repo include root.

# cc_library / cc_test are native (autoloaded) rules in Bazel 8.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    hdrs = [
        "B2.h",
        "B2c.h",
        "core/drone_types.h",
        "core/fsw.h",
        "core/fswtime.h",
        "runtime.h",
        "static_vector.h",
    ],
    strip_include_prefix = "/vehicle",
)

_L0_TESTS = {
    "drone_types_test": "core/drone_types_test.cc",
    "fsw_test": "core/fsw_test.cc",
    "fswtime_test": "core/fswtime_test.cc",
    "b2_test": "b2_test.cc",
    "runtime_test": "runtime_test.cc",
    "static_vector_test": "static_vector_test.cc",
}

[
    cc_test(
        name = name,
        srcs = [src],
        copts = ["-std=c++17"],
        deps = [":core"],
    )
    for name, src in _L0_TESTS.items()
]
