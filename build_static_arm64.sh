#!/usr/bin/env bash
#
# Cross-compile the CMSIS-DAP TCP wrapper as a static arm64 binary using an
# Alpine (musl) Docker container running under QEMU emulation. This avoids
# needing a glibc aarch64 cross-toolchain installed on the host, and musl
# makes fully static linking straightforward.
#
# Requires docker with arm64 emulation (QEMU) support -- the QEMU binfmt
# handler is registered automatically below if it isn't already (requires a
# one-time --privileged container run; see register_binfmt()).
#
# The actual compile is `make static` (see Makefile), which links with -static
# and passes -s to strip symbols at link time: Alpine's static musl
# libc/libstdc++ archives carry embedded debug info, which roughly doubles
# the output size if left in (static linking already embeds the whole
# runtime, unlike a dynamic build).
#
# Override the image or output name if needed:
#   ALPINE_IMAGE=alpine:3.19 ./build_static_arm64.sh
#   OUT=my_binary ./build_static_arm64.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${OUT:-${SCRIPT_DIR}/cmsis_dap_tcp}"
ALPINE_IMAGE="${ALPINE_IMAGE:-alpine:3.20}"
TARGET_PLATFORM="${TARGET_PLATFORM:-linux/arm64}"

if ! command -v docker >/dev/null 2>&1; then
    echo "error: docker not found." >&2
    exit 1
fi

register_binfmt() {
    # Skip if the kernel already has a qemu-aarch64 (or equivalent)
    # binfmt_misc handler registered -- Docker Desktop ships this out of
    # the box, some Linux hosts have it from other tooling.
    if [ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
        return
    fi
    if docker run --rm --platform "$TARGET_PLATFORM" "$ALPINE_IMAGE" true >/dev/null 2>&1; then
        return
    fi
    echo ">> registering QEMU binfmt handlers for arm64 (one-time, needs --privileged)"
    docker run --privileged --rm tonistiigi/binfmt --install arm64 >/dev/null
}

register_binfmt

# Objects go to a separate directory so an arm64 build does not collide with a
# host build in ./build.
echo ">> compiling -> ${OUT} (${ALPINE_IMAGE}, ${TARGET_PLATFORM}, musl)"

docker run --rm \
    --platform "$TARGET_PLATFORM" \
    -e HOST_UID="$(id -u)" \
    -e HOST_GID="$(id -g)" \
    -v "${SCRIPT_DIR}:/src" \
    -w /src \
    "${ALPINE_IMAGE}" \
    sh -eu -c '
        apk add --no-cache g++ make musl-dev >/dev/null
        make static BUILD_DIR=build-arm64-musl OUT=.build_out

        # container runs as root, so the binary and object dir would
        # otherwise end up root-owned on the host; hand them back to the
        # invoking user.
        chown -R "$HOST_UID:$HOST_GID" .build_out build-arm64-musl
    '

mv "${SCRIPT_DIR}/.build_out" "${OUT}"

echo ">> wrote ${OUT}"
file "${OUT}" 2>/dev/null || true
