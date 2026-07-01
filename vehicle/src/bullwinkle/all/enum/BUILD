# Enum reflection metadata (build-phase / diagnostic, not a runtime hot path).
# `strip_include_prefix = "/vehicle"` exposes headers as `src/bullwinkle/all/enum/...`.
# cc_library / cc_test are native (autoloaded) rules in Bazel 8.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "symbol_table",
    srcs = ["SymbolTable.cc"],
    hdrs = ["SymbolTable.h"],
    strip_include_prefix = "/vehicle",
    copts = ["-std=c++17"],
    deps = ["//vehicle/src/bullwinkle/all:core"],
)

cc_test(
    name = "symbol_table_test",
    srcs = ["SymbolTable_test.cc"],
    copts = ["-std=c++17"],
    deps = [":symbol_table"],
)
