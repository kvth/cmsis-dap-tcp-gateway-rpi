# Static arm64 CMSIS-DAP TCP wrapper (musl libc), built from this checkout in
# Alpine. Driven by build_static_arm64.sh, which picks the platform and exports
# the "out" stage; it can also be used directly:
#
#   podman build --platform linux/arm64 --target out --output type=local,dest=. .
#
# This avoids needing a glibc aarch64 cross-toolchain installed on the host,
# and musl makes fully static linking straightforward.
#
# The actual compile is `make static` (see Makefile), which links with -static
# and passes -s to strip symbols at link time: Alpine's static musl
# libc/libstdc++ archives carry embedded debug info, which roughly doubles the
# output size if left in (static linking already embeds the whole runtime,
# unlike a dynamic build).

# Default, overridable with --build-arg.
ARG ALPINE_IMAGE=docker.io/library/alpine:3.20

FROM ${ALPINE_IMAGE} AS build

RUN apk add --no-cache g++ make musl-dev

# Only what `make static` needs, so host build output (build/, a host-built
# cmsis_dap_tcp, tests/) never reaches the image and never busts the cache.
WORKDIR /src
COPY Makefile *.cpp *.h ./
RUN make static

# --output type=local writes this as <dest>/cmsis_dap_tcp, owned by the
# invoking user under rootless podman.
FROM scratch AS out
COPY --from=build /src/cmsis_dap_tcp /cmsis_dap_tcp
