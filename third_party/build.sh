#!/usr/bin/env bash
# Build libfranka + runtime deps for a target platform via Docker buildx.
# Usage: third_party/build.sh linux/arm64
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

echo ">>> Building libfranka $LIBFRANKA_TAG for $PLATFORM into $OUT_DIR"

# Make sure a buildx builder with multi-arch (qemu) support exists.
if ! docker buildx inspect viam-franka-builder >/dev/null 2>&1; then
    docker buildx create --name viam-franka-builder --use
fi

mkdir -p "$OUT_DIR"

docker buildx build \
    --platform "$PLATFORM" \
    --build-arg "LIBFRANKA_TAG=$LIBFRANKA_TAG" \
    --target export \
    --output "type=local,dest=$OUT_DIR" \
    -f "$REPO_ROOT/third_party/Dockerfile" \
    "$REPO_ROOT/third_party"

echo ">>> Done. Headers in $OUT_DIR/include, libs in $OUT_DIR/lib"
ls -1 "$OUT_DIR/lib" 2>/dev/null | head -20 || true
