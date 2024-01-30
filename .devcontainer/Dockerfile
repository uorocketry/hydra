FROM mcr.microsoft.com/devcontainers/rust:0-1-bullseye

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y && apt-get install -y cmake

# Install ARM GCC deps
RUN mkdir -p toolchain && \
    curl -L "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz" \
    | tar --strip-components=1 -xJ -C toolchain
ENV PATH="${PATH}:/toolchain/bin"