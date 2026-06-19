#!/usr/bin/env bash
#
# Install the *system-level* build prerequisites for the Drone control stack.
#
# Philosophy: this repo builds with Bazel, and all heavyweight C++ libraries
# (gRPC, protobuf, abseil, BoringSSL, c-ares, re2, zlib, upb) are resolved
# hermetically through MODULE.bazel. They are intentionally NOT installed here.
#
# The only things that genuinely come from the OS are:
#   1. A C/C++ compiler  -- Bazel's cc_* rules drive the host toolchain.
#   2. libcurl headers   -- the sources include <curl/curl.h> directly, and we
#                           deliberately keep curl as a system lib (a hermetic
#                           curl+TLS build is more trouble than it is worth).
#   3. git               -- used by the toolchain / developer workflow.
#   4. Bazelisk          -- launches the exact Bazel version in .bazelversion.
#
# The script is idempotent and supports dnf (RHEL/CentOS/Fedora) and apt
# (Debian/Ubuntu). Run it as a normal user; it uses sudo only where required.
#
set -euo pipefail

BAZELISK_VERSION="v1.27.0"

log()  { printf '\033[1;34m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[install]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

# Run a command with sudo only when we are not already root.
as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then "$@"; else sudo "$@"; fi
}

install_with_dnf() {
  log "Detected dnf. Installing toolchain + libcurl headers..."
  # CRB carries some -devel packages on CentOS Stream / RHEL; harmless elsewhere.
  as_root dnf install -y --setopt=install_weak_deps=False \
    --enablerepo=crb \
    clang \
    libcurl-devel \
    git \
    || as_root dnf install -y clang libcurl-devel git
}

install_with_apt() {
  log "Detected apt. Installing toolchain + libcurl headers..."
  as_root apt-get update
  as_root apt-get install -y --no-install-recommends \
    clang \
    libcurl4-openssl-dev \
    git \
    ca-certificates \
    curl
}

install_bazelisk() {
  if command -v bazel >/dev/null 2>&1; then
    log "bazel already on PATH ($(command -v bazel)); skipping bazelisk install."
    return
  fi
  local arch url
  case "$(uname -m)" in
    x86_64|amd64) arch="amd64" ;;
    aarch64|arm64) arch="arm64" ;;
    *) die "Unsupported architecture for bazelisk: $(uname -m)" ;;
  esac
  url="https://github.com/bazelbuild/bazelisk/releases/download/${BAZELISK_VERSION}/bazelisk-linux-${arch}"
  log "Installing bazelisk ${BAZELISK_VERSION} (${arch}) -> /usr/local/bin/bazel"
  local tmp; tmp="$(mktemp)"
  curl -fsSL "$url" -o "$tmp"
  as_root install -m 0755 "$tmp" /usr/local/bin/bazel
  rm -f "$tmp"
}

main() {
  if command -v dnf >/dev/null 2>&1; then
    install_with_dnf
  elif command -v apt-get >/dev/null 2>&1; then
    install_with_apt
  else
    die "No supported package manager found (need dnf or apt-get)."
  fi

  install_bazelisk

  log "Done. Verifying:"
  clang --version | head -1
  bazel --version 2>/dev/null || warn "run 'bazel version' inside the repo to fetch the pinned Bazel"
  log "Hermetic C++ deps (gRPC/protobuf/abseil/...) are fetched by Bazel on first build."
}

main "$@"
