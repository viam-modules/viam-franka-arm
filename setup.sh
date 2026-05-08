#!/usr/bin/env bash
# Cloud-build setup: install system deps, build libfranka 0.9.2 natively,
# and stage the result under third_party/<triple>/ so `make module` can
# link against it.
#
# Runs once per Viam cloud-build invocation. Each build runner is already
# the target architecture, so we do a plain native cmake build — no
# cross-compilation, no docker-in-docker.
set -euo pipefail

LIBFRANKA_TAG="${LIBFRANKA_TAG:-0.9.2}"
JOBS="${JOBS:-$(nproc)}"

UNAME_M="$(uname -m)"
case "$UNAME_M" in
    aarch64|arm64) TRIPLE="linux-arm64" ;;
    x86_64|amd64)  TRIPLE="linux-amd64" ;;
    *) echo "Unsupported arch: $UNAME_M" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="$REPO_ROOT/third_party/$TRIPLE"
BUILD_DIR="$REPO_ROOT/third_party/build/$TRIPLE"

# Skip if already built (idempotent for local re-runs).
if [ -f "$OUT_DIR/lib/libfranka.so" ] && [ -d "$OUT_DIR/include/franka" ]; then
    echo ">>> third_party/$TRIPLE already populated; skipping rebuild."
    exit 0
fi

echo ">>> Installing build deps via apt."
SUDO=""; command -v sudo >/dev/null 2>&1 && [ "$(id -u)" -ne 0 ] && SUDO=sudo
export DEBIAN_FRONTEND=noninteractive
$SUDO apt-get update -y
$SUDO apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libeigen3-dev \
    libpoco-dev \
    patchelf \
    pkg-config

echo ">>> Cloning libfranka $LIBFRANKA_TAG."
mkdir -p "$BUILD_DIR"
if [ ! -d "$BUILD_DIR/libfranka" ]; then
    git clone --recursive --branch "$LIBFRANKA_TAG" \
        https://github.com/frankaemika/libfranka.git "$BUILD_DIR/libfranka"
fi

echo ">>> Configuring + building libfranka into $OUT_DIR."
mkdir -p "$BUILD_DIR/libfranka/build" "$OUT_DIR/lib" "$OUT_DIR/include"
(
    cd "$BUILD_DIR/libfranka/build"
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TESTS=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DCMAKE_INSTALL_PREFIX="$OUT_DIR" \
          ..
    cmake --build . -j"$JOBS"
    cmake --install .
)

echo ">>> Capturing runtime model library blob (loaded via dlopen at runtime)."
find "$BUILD_DIR/libfranka" -name 'libfrankamodel*.so*' -exec cp -av {} "$OUT_DIR/lib/" \; || true

echo ">>> Capturing Poco shared libs (libfranka links against them)."
for libname in PocoFoundation PocoNet PocoUtil PocoXML PocoJSON; do
    for so in $(find /usr -name "lib${libname}.so*" 2>/dev/null); do
        cp -a "$so" "$OUT_DIR/lib/"
    done
done

echo ">>> third_party/$TRIPLE staged. Headers + libs:"
ls -1 "$OUT_DIR/include/franka" | head -5 || true
echo "..."
ls -1 "$OUT_DIR/lib"
