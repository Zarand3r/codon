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
        "core/util.h",
        "runtime.h",
        "static_vector.h",
    ],
    strip_include_prefix = "/vehicle",
)

_L0_TESTS = {
    "drone_types_test": "core/drone_types_test.cc",
    "fsw_test": "core/fsw_test.cc",
    "fswtime_test": "core/fswtime_test.cc",
    "util_test": "core/util_test.cc",
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

# AlignedBuffer — per-shard heap storage (has a .cc; depends on :core for util/fsw).
cc_library(
    name = "aligned_buffer",
    srcs = ["AlignedBuffer.cc"],
    hdrs = ["core/AlignedBuffer.h"],
    strip_include_prefix = "/vehicle",
    copts = ["-std=c++17"],
    deps = [":core"],
)

cc_test(
    name = "aligned_buffer_test",
    srcs = ["aligned_buffer_test.cc"],
    copts = ["-std=c++17"],
    deps = [":aligned_buffer"],
)

# slate_enums — shard set, element-access policy, permission algebra + reflection.
cc_library(
    name = "slate_enums",
    srcs = ["slate_enums.cc"],
    hdrs = ["slate_enums.h"],
    strip_include_prefix = "/vehicle",
    copts = ["-std=c++17"],
    deps = [
        ":core",
        "//vehicle/src/bullwinkle/all/enum:symbol_table",
    ],
)

cc_test(
    name = "slate_enums_test",
    srcs = ["slate_enums_test.cc"],
    copts = ["-std=c++17"],
    deps = [":slate_enums"],
)

# slate_id — the packed slate_element_t (header-only; shifts + masks, no hot-path calls).
cc_library(
    name = "slate_id",
    hdrs = ["slate_id.h"],
    strip_include_prefix = "/vehicle",
    deps = [":slate_enums"],
)

cc_test(
    name = "slate_id_test",
    srcs = ["slate_id_test.cc"],
    copts = ["-std=c++17"],
    deps = [":slate_id"],
)

# slate_type — deterministic cross-string type id (header-only; hashes a per-type name).
cc_library(
    name = "slate_type",
    hdrs = ["slate_type.h"],
    strip_include_prefix = "/vehicle",
    deps = [
        ":core",
        "//vehicle/src/hash:hash",
    ],
)

cc_test(
    name = "slate_type_test",
    srcs = ["slate_type_test.cc"],
    copts = ["-std=c++17"],
    deps = [":slate_type"],
)

# slate_info — umbrella header the Slate consumers include (ids + type id + fsw).
cc_library(
    name = "slate_info",
    hdrs = [
        "slate_info.h",
        "slate_tokens.h",
    ],
    strip_include_prefix = "/vehicle",
    deps = [
        ":core",
        ":slate_id",
        ":slate_type",
    ],
)

# Consumer-compile gate: compiles the imported slate_tokens.h consumer object-only
# (cc_library never links an executable, so the not-yet-defined accountant / fsw-logging
# symbols are fine). A mis-inferred contract fails to build here. See IMPLEMENTATION_PLAN §4.
cc_library(
    name = "slate_tokens_compile",
    srcs = ["slate_tokens_compile_test.cc"],
    copts = ["-std=c++17"],
    deps = [":slate_info"],
)
