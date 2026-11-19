# syntax=docker/dockerfile:1.7
# ferroute — multi-stage build. Targets: base, tools, dev, builder, runtime.
# Every base image is pinned by tag AND digest (Dependabot's docker ecosystem bumps both).

############################ base ############################
FROM rust:1.100.0-slim-trixie@sha256:6f14b2b4b278ec0d50aa1d69541c304ce0f95e0f6268487aa9924af1c2e9aebe AS base
ENV DEBIAN_FRONTEND=noninteractive \
    CARGO_TERM_COLOR=always \
    CARGO_INCREMENTAL=0 \
    RUST_BACKTRACE=1
# mesa-vulkan-drivers provides lavapipe (software Vulkan) so wgpu has an adapter in CI/containers.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates git pkg-config make python3 musl-tools \
        mesa-vulkan-drivers libvulkan1
RUN rustup component add rustfmt clippy \
    && rustup target add x86_64-unknown-linux-musl

############################ tools ############################
FROM base AS tools
ARG CARGO_DENY_VERSION=0.21.0
ARG CARGO_AUDIT_VERSION=0.23.0
ARG CARGO_LLVM_COV_VERSION=0.9.1
ARG CARGO_NEXTEST_VERSION=0.9.152
ARG GIT_CLIFF_VERSION=2.14.2
RUN rustup component add llvm-tools-preview
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/usr/local/cargo/git \
    --mount=type=cache,target=/tmp/tools-target \
    CARGO_TARGET_DIR=/tmp/tools-target cargo install --locked \
        cargo-deny@${CARGO_DENY_VERSION} \
        cargo-audit@${CARGO_AUDIT_VERSION} \
        cargo-llvm-cov@${CARGO_LLVM_COV_VERSION} \
        cargo-nextest@${CARGO_NEXTEST_VERSION} \
        git-cliff@${GIT_CLIFF_VERSION}

############################ dev ############################
FROM base AS dev
COPY --from=tools /usr/local/cargo/bin/cargo-deny \
                  /usr/local/cargo/bin/cargo-audit \
                  /usr/local/cargo/bin/cargo-llvm-cov \
                  /usr/local/cargo/bin/cargo-nextest \
                  /usr/local/cargo/bin/git-cliff \
                  /usr/local/cargo/bin/
# llvm-tools-preview lives in the rustup toolchain, which is world-readable.
RUN rustup component add llvm-tools-preview \
    && chmod -R a+rwX /usr/local/cargo /usr/local/rustup \
    && git config --system --add safe.directory /work
# Run as any uid: `docker run -u $(id -u):$(id -g) -e HOME=/tmp ...` (see Makefile RUN macro).
ENV CARGO_HOME=/usr/local/cargo \
    WGPU_BACKEND=vulkan
WORKDIR /work
CMD ["bash"]

############################ builder ############################
FROM base AS builder
WORKDIR /src
COPY . .
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/usr/local/cargo/git \
    --mount=type=cache,target=/src/target \
    cargo build --release --locked -p ferroute-cli \
    && mkdir -p /out && cp target/release/ferroute /out/ferroute

############################ runtime ############################
FROM debian:trixie-slim@sha256:a72b4433de1cf7d6bae0c4cee74d0547f58601a643b3375f070c51600d63b6b2 AS runtime
ARG VERSION=dev
ARG REVISION=unknown
LABEL org.opencontainers.image.title="ferroute" \
      org.opencontainers.image.description="Rust PCB autorouter for KiCad 10/11" \
      org.opencontainers.image.licenses="AGPL-3.0-only" \
      org.opencontainers.image.source="https://github.com/vanhanit/ferroute" \
      org.opencontainers.image.url="https://github.com/vanhanit/ferroute" \
      org.opencontainers.image.version="${VERSION}" \
      org.opencontainers.image.revision="${REVISION}"
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --system --uid 10001 --create-home --home-dir /home/ferroute ferroute
COPY --from=builder /out/ferroute /usr/local/bin/ferroute
USER ferroute
WORKDIR /data
ENTRYPOINT ["ferroute"]
CMD ["--help"]
