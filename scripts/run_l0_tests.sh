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
)

# Consumer-compile gates (IMPLEMENTATION_PLAN §4): compile an *imported* consumer
# object-only to validate our inferred contracts as real code uses them. Compiled
# (`-c`, no link/run) because these consumers' link deps (accountant globals, the
# fsw logging layer) do not exist yet; a mis-inferred contract still fails to compile.
COMPILE_ONLY=(
  "vehicle/src/bullwinkle/all/slate_tokens_compile_test.cc"
  "vehicle/src/bullwinkle/all/slate_tokens.cc"
)

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

total=$(( ${#TESTS[@]} + ${#COMPILE_ONLY[@]} ))
if [ "$fail" -eq 0 ]; then
  echo "GATE GREEN — ${#TESTS[@]} test(s) + ${#COMPILE_ONLY[@]} consumer-compile(s) = ${total} checks passed"
else
  echo "GATE RED"
fi
exit "$fail"
