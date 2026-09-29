#!/usr/bin/env bash
#
# Cross-compile the CMSIS-DAP TCP wrapper as a static arm64 binary with podman,
# as described by the Containerfile (layers are cached, so repeat builds are
# fast). The build runs in an Alpine (musl) container under QEMU, which a
# rootless podman cannot register itself -- on Debian/Ubuntu:
# sudo apt install qemu-user-static binfmt-support
#
#   ./build_static_arm64.sh            # writes ./cmsis_dap_tcp
#   ./build_static_arm64.sh /some/dir  # writes /some/dir/cmsis_dap_tcp

set -euo pipefail

cd "$(dirname "$0")"

OUT_DIR=${1:-.}

podman build --platform linux/arm64 --target out --output "type=local,dest=$OUT_DIR" . || {
    echo "error: build failed. If it could not run arm64 containers -- no QEMU binfmt" >&2
    echo "       handler registered -- install one, e.g. sudo apt install qemu-user-static binfmt-support" >&2
    exit 1
}

file "$OUT_DIR/cmsis_dap_tcp"
