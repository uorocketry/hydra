# Base image for toolchain download
FROM mcr.microsoft.com/devcontainers/rust:latest as toolchain

RUN mkdir -p /toolchain && \
    curl -L "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz" \
    | tar --strip-components=1 -xJ -C /toolchain

# Final image
FROM mcr.microsoft.com/devcontainers/rust:latest

# Copy the toolchain from the previous stage
COPY --from=toolchain /toolchain /toolchain

ENV PATH="${PATH}:/toolchain/bin"
ARG DEBIAN_FRONTEND=noninteractive

# Necessary system packages
RUN apt-get update && apt-get install -y cmake pkg-config libusb-1.0-0-dev libftdi1-dev libudev-dev libssl-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Rust crates
RUN cargo install probe-rs --features cli \
    && cargo install cargo-make