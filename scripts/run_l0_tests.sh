#!/usr/bin/env bash
# Gate: compile + run every core/storage unit test with the repo's include root
# (`vehicle/`). A failed compile is also the include-resolution check (an unresolved
# internal include fails here). Exits 0 iff all tests pass.
#
# Each TESTS entry is a space-separated source list: the test .cc first (used for the
# binary name), then any extra .cc it must link (e.g. AlignedBuffer.cc).
#
# g++ is the current driver (zero external deps, deterministic, no network); the
# Bazel cc_test targets are the eventual driver once the dependency toolchain lands.

set -u -o pipefail
cd "$(dirname "$0")/.." || exit 2

CXX="${CXX:-g++}"
STD="-std=c++17"
WARN="-Wall -Wextra -Werror"
INC="-I vehicle"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

TESTS=(
  "vehicle/src/bullwinkle/all/core/drone_types_test.cc"
  "vehicle/src/bullwinkle/all/core/fsw_test.cc"
  "vehicle/src/bullwinkle/all/core/fsw_log_test.cc"
  "vehicle/src/bullwinkle/all/core/fswtime_test.cc"
  "vehicle/src/bullwinkle/all/core/util_test.cc"
  "vehicle/src/hash/hash_test.cc"
  "vehicle/src/bullwinkle/all/b2_test.cc"
  "vehicle/src/bullwinkle/all/runtime_test.cc"
  "vehicle/src/bullwinkle/all/static_vector_test.cc"
  "vehicle/src/bullwinkle/all/aligned_buffer_test.cc vehicle/src/bullwinkle/all/AlignedBuffer.cc"
  "vehicle/src/bullwinkle/all/enum/SymbolTable_test.cc vehicle/src/bullwinkle/all/enum/SymbolTable.cc"
  "vehicle/src/bullwinkle/all/slate_enums_test.cc vehicle/src/bullwinkle/all/slate_enums.cc vehicle/src/bullwinkle/all/enum/SymbolTable.cc"
  "vehicle/src/bullwinkle/all/slate_id_test.cc"
  "vehicle/src/bullwinkle/all/slate_type_test.cc"
  "vehicle/src/bullwinkle/all/slate_info_test.cc"
  "vehicle/src/bullwinkle/all/slate_path_map_test.cc"
  "vehicle/src/bullwinkle/all/handle_test.cc"
  "vehicle/src/bullwinkle/all/slate_accessor_test.cc"
)

# Consumer-compile gates (IMPLEMENTATION_PLAN §4): compile an *imported* consumer
# object-only to validate our inferred contracts as real code uses them. Compiled
# (`-c`, no link/run) because these consumers' link deps (accountant globals, the
# fsw logging layer) do not exist yet; a mis-inferred contract still fails to compile.
COMPILE_ONLY=(
  "vehicle/src/bullwinkle/all/slate_tokens_compile_test.cc"
  "vehicle/src/bullwinkle/all/slate_tokens.cc"
  "vehicle/src/bullwinkle/all/slate_enum_headers_compile_test.cc"
)

# Relaxed consumer-compile: imported .cc that carry pre-existing diagnostic quirks we
# will not edit — `%hu` format on a 64-bit id (lossy low-16-bits print in an error
# string, not a correctness issue) and unused params in imported stub bodies. We still
# type-check the whole translation unit against our contracts; we just relax -Wformat/
# -Wunused-parameter on these imported diagnostics.
GOLDEN="vehicle/src/bullwinkle/all/slate_golden_path_test.cc vehicle/src/bullwinkle/all/SlateBuilderStore.cc vehicle/src/bullwinkle/all/SlateMemory.cc vehicle/src/bullwinkle/all/SlateLayout.cc vehicle/src/bullwinkle/all/SlateBuilder.cc vehicle/src/bullwinkle/all/Slate.cc vehicle/src/bullwinkle/all/slate_tokens.cc vehicle/src/bullwinkle/all/AlignedBuffer.cc vehicle/src/bullwinkle/all/slate_enums.cc vehicle/src/bullwinkle/all/enum/SymbolTable.cc"
COMPILE_ONLY_RELAXED=(
  "vehicle/src/bullwinkle/all/SlateLayout.cc"
  "vehicle/src/bullwinkle/all/SlateBuilder.cc"
  "vehicle/src/bullwinkle/all/Slate.cc"
  "vehicle/src/bullwinkle/all/SlateMemory.cc"
)
RELAX="-Wno-format -Wno-unused-parameter"

fail=0
for entry in "${TESTS[@]}"; do
  first="${entry%% *}"                       # test .cc (first token) -> binary name
  bin="$TMP/$(basename "$first" .cc)"
  # shellcheck disable=SC2086  # $entry is an intentional multi-file source list
  if ! $CXX $STD $WARN $INC -o "$bin" $entry 2> "$TMP/err"; then
    echo "COMPILE FAIL: $first"; sed 's/^/    /' "$TMP/err"; fail=1; continue
  fi
  if ! "$bin"; then
    echo "RUN FAIL: $first"; fail=1
  fi
done

for src in "${COMPILE_ONLY[@]}"; do
  if ! $CXX $STD $WARN $INC -fsyntax-only "$src" 2> "$TMP/err"; then
    echo "CONSUMER-COMPILE FAIL: $src"; sed 's/^/    /' "$TMP/err"; fail=1
  fi
done

for src in "${COMPILE_ONLY_RELAXED[@]}"; do
  # shellcheck disable=SC2086
  if ! $CXX $STD $WARN $RELAX $INC -fsyntax-only "$src" 2> "$TMP/err"; then
    echo "CONSUMER-COMPILE FAIL (relaxed): $src"; sed 's/^/    /' "$TMP/err"; fail=1
  fi
done

# Golden-path L1 integration (links + runs the whole Slate stack).
# shellcheck disable=SC2086
if $CXX $STD -Wno-format -Wno-unused-parameter $INC -o "$TMP/golden" $GOLDEN 2> "$TMP/err" && "$TMP/golden" > /dev/null 2>&1; then
  golden=1
else
  echo "GOLDEN-PATH FAIL"; sed 's/^/    /' "$TMP/err" | head -10; fail=1; golden=0
fi

n_consumer=$(( ${#COMPILE_ONLY[@]} + ${#COMPILE_ONLY_RELAXED[@]} ))
total=$(( ${#TESTS[@]} + n_consumer ))
if [ "$fail" -eq 0 ]; then
  echo "GATE GREEN — ${#TESTS[@]} test(s) + ${n_consumer} consumer-compile(s) = ${total} checks + golden-path(${golden}) passed"
else
  echo "GATE RED"
fi
exit "$fail"
