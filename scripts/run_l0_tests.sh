#!/usr/bin/env bash
# P0 gate: compile + run every L0 (core primitive) unit test with the repo's
# include root (`vehicle/`). A failed compile is also the include-resolution check
# for L0 (an unresolved internal include fails here). Exits 0 iff all tests pass.
#
# g++ is the current driver (zero external deps, deterministic, no network); the
# Bazel `cc_test` targets in the BUILD files are the eventual driver once the
# dependency-fetching toolchain (D1/D3) is wired.

set -u -o pipefail
cd "$(dirname "$0")/.." || exit 2

CXX="${CXX:-g++}"
STD="-std=c++17"
WARN="-Wall -Wextra -Werror"
INC="-I vehicle"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

TESTS=(
  vehicle/src/bullwinkle/all/core/drone_types_test.cc
  vehicle/src/bullwinkle/all/core/fsw_test.cc
  vehicle/src/bullwinkle/all/core/fswtime_test.cc
  vehicle/src/bullwinkle/all/core/util_test.cc
  vehicle/src/hash/hash_test.cc
  vehicle/src/bullwinkle/all/b2_test.cc
  vehicle/src/bullwinkle/all/runtime_test.cc
  vehicle/src/bullwinkle/all/static_vector_test.cc
)

fail=0
for t in "${TESTS[@]}"; do
  bin="$TMP/$(basename "$t" .cc)"
  if ! $CXX $STD $WARN $INC -o "$bin" "$t" 2> "$TMP/err"; then
    echo "COMPILE FAIL: $t"; sed 's/^/    /' "$TMP/err"; fail=1; continue
  fi
  if ! "$bin"; then
    echo "RUN FAIL: $t"; fail=1
  fi
done

if [ "$fail" -eq 0 ]; then
  echo "L0 GATE GREEN — $((${#TESTS[@]})) test(s) passed"
else
  echo "L0 GATE RED"
fi
exit "$fail"
