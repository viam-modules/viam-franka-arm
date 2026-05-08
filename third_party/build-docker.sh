#!/usr/bin/env bash
# Build libfranka + runtime deps for a target platform.
# Usage: third_party/build.sh linux/arm64
#
# Detects whether the host has docker+buildx or podman and uses the right
# command. For cross-arch builds (e.g. arm64 host on amd64), QEMU binfmt
# emulation must be registered:
#   docker:  docker run --rm --privileged tonistiigi/binfmt --install all
#   podman:  sudo dnf install qemu-user-static  (Fedora/RHEL)
#            sudo apt install qemu-user-static  (Debian/Ubuntu)
set -euo pipefail

PLATFORM="${1:-linux/arm64}"
LIBFRANKA_TAG="${LIBFRANKA_TAG:-0.9.2}"

case "$PLATFORM" in
    linux/arm64)  TRIPLE="linux-arm64"  ;;
    linux/amd64)  TRIPLE="linux-amd64"  ;;
    *) echo "Unsupported platform: $PLATFORM" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="$REPO_ROOT/third_party/$TRIPLE"
mkdir -p "$OUT_DIR"

echo ">>> Building libfranka $LIBFRANKA_TAG for $PLATFORM into $OUT_DIR"

# Detect engine. The 'docker' command on this host may be a podman shim;
# treat that as podman so we use podman-native flags.
if command -v podman >/dev/null 2>&1 && \
   { ! command -v docker >/dev/null 2>&1 || \
     docker --version 2>/dev/null | grep -qi podman; }; then
    ENGINE="podman"
elif command -v docker >/dev/null 2>&1; then
    ENGINE="docker"
else
    echo "Neither docker nor podman found." >&2
    exit 3
fi
echo ">>> Using container engine: $ENGINE"

case "$ENGINE" in
    docker)
        if ! docker buildx inspect viam-franka-builder >/dev/null 2>&1; then
            docker buildx create --name viam-franka-builder --use
        fi
        docker buildx build \
            --platform "$PLATFORM" \
            --build-arg "LIBFRANKA_TAG=$LIBFRANKA_TAG" \
            --target export \
            --output "type=local,dest=$OUT_DIR" \
            -f "$REPO_ROOT/third_party/Dockerfile" \
            "$REPO_ROOT/third_party"
        ;;
    podman)
        # podman supports --platform and --output type=local natively.
        podman build \
            --platform="$PLATFORM" \
            --build-arg "LIBFRANKA_TAG=$LIBFRANKA_TAG" \
            --target=export \
            --output "type=local,dest=$OUT_DIR" \
            -f "$REPO_ROOT/third_party/Dockerfile" \
            "$REPO_ROOT/third_party"
        ;;
esac

echo ">>> Done. Headers in $OUT_DIR/include, libs in $OUT_DIR/lib"
ls -1 "$OUT_DIR/lib" 2>/dev/null | head -20 || true
