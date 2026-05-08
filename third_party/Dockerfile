# syntax=docker/dockerfile:1.6
#
# Builds libfranka 0.9.2 (last release with Panda support) and its runtime
# dependencies inside a target-arch container, then exports headers + .so files
# to /artifacts. Driven by third_party/build.sh via `docker buildx build`.
#
# Output layout (consumed by the parent Makefile):
#   /artifacts/include/franka/...
#   /artifacts/lib/libfranka.so*
#   /artifacts/lib/libPocoNet.so*  (and other runtime poco/eigen .so files)
#
ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION} AS build

ARG LIBFRANKA_TAG=0.9.2
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        git \
        libeigen3-dev \
        libpoco-dev \
        patchelf \
        pkg-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src

RUN git clone --recursive --branch ${LIBFRANKA_TAG} https://github.com/frankaemika/libfranka.git

WORKDIR /src/libfranka/build

RUN cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TESTS=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DCMAKE_INSTALL_PREFIX=/artifacts \
          .. \
 && cmake --build . -j"$(nproc)" \
 && cmake --install .

# libfranka 0.9.x loads libfrankamodel.<platform>.so at runtime via dlopen.
# Copy any model libraries shipped under the build tree into /artifacts/lib so
# they ride along with libfranka.so.
RUN find /src/libfranka -name 'libfrankamodel*.so*' -exec cp -av {} /artifacts/lib/ \; || true

# Runtime Poco shared libs need to ride along too — libfranka.so links to them.
# Also bundle libpcre because Poco 80 (Ubuntu 22.04) needs libpcre.so.3 (pcre1)
# while many recent hosts only ship pcre2.
RUN set -e; mkdir -p /artifacts/lib; \
    for libname in PocoFoundation PocoNet PocoUtil PocoXML PocoJSON pcre; do \
        for so in $(find /usr -name "lib${libname}.so*" 2>/dev/null); do \
            cp -a "$so" /artifacts/lib/; \
        done; \
    done

# Final stage: just the artifacts, ready for `docker buildx build --output`.
FROM scratch AS export
COPY --from=build /artifacts /
