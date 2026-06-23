#!/usr/bin/env bash
# Verification gate for humans and the autonomous harness.
#
# Contract (claude-skills docs/AGENT_HARNESS.md §8): build + test, pipe ALL output
# to a log, exit 0 on green / non-zero on red.
#
# Authoritative gate = the L0 unit tests via g++ (zero external deps, deterministic,
# no network). The g++ compile is also the include-resolution check. Bazel is the
# eventual driver; it is attempted best-effort (the full //... graph won't build
# until the missing layers + dependency toolchain land — see ROADMAP/IMPLEMENTATION_PLAN).

set -u -o pipefail
cd "$(dirname "$0")/.." || exit 2

LOG="${CHECK_LOG:-/tmp/codon-check.log}"
: >"$LOG"

echo "== L0 unit tests (authoritative) ==" | tee -a "$LOG"
if ! ./scripts/run_l0_tests.sh 2>&1 | tee -a "$LOG"; then
  echo "GATE RED: L0 tests failed (see $LOG)"
  exit 1
fi

if command -v bazel >/dev/null 2>&1; then
  echo "== bazel build L0 (best-effort) ==" | tee -a "$LOG"
  if bazel build //vehicle/src/bullwinkle/all:core //vehicle/src/hash:hash \
      >>"$LOG" 2>&1; then
    echo "bazel L0 build: OK" | tee -a "$LOG"
  else
    echo "WARN: bazel L0 build unavailable (offline dep fetch?); g++ gate is authoritative." \
      | tee -a "$LOG"
  fi
fi

echo "GATE GREEN"
exit 0
