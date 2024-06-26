# HYDRA &emsp; [![Build Status]][actions] [![docs-badge]][docs]

***HY**per **D**ynamic **R**ocketry **A**vionics*

[Build Status]: https://github.com/uorocketry/hydra/actions/workflows/build.yml/badge.svg
[actions]: https://github.com/uorocketry/hydra/actions?query=branch%3Amaster
[docs-badge]: https://img.shields.io/github/actions/workflow/status/uorocketry/hydra/docs.yml?label=docs
[docs]: http://hydra-docs.uorocketry.ca/common_arm

**uORocketry's next-generation avionics system**

---

## Getting Started

> If you are in a DevContainer skip to step 5.

- Install Rust: https://www.rust-lang.org/tools/install
- Install necessary build tools:
  - cargo-make: `cargo install cargo-make`
  - probe-rs: `cargo install probe-rs --features cli`
- Install the [ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (last tested with 13.2) and have it available in your PATH
  - Arch Linux: `sudo pacman -S arm-none-eabi-gcc`
  - Alpine/Debian/Ubuntu: <https://pkgs.org/download/gcc-arm-none-eabi>
  - MacOS: `brew install arm-none-eabi-gcc`
- Build: `cargo build`
  - In case it fails, try `cargo build --release`
- Run tests:
  - In the host machine: `cargo make test-host`
  - In the device: `cargo make test-device`
- Flash on hardware: `cargo run --bin main`

For more detailed instructions on flashing, debugging, and more, please see [the wiki](https://avwiki.uorocketry.ca/en/Avionics/HYDRA/Software).

## Windows Users

### Setting up with Docker

- Install [Docker](https://docs.docker.com/desktop/install/windows-install/)
- Install [VS Code](https://code.visualstudio.com/download)
- From VS Code, install the "Dev Containers" extension
- press `ctrl` + `shift` + `p`, and search for `Dev Containers: Open Folder in Container`

### Setting up with WSL

- Enable WSL: https://learn.microsoft.com/en-us/windows/wsl/install
- Install a linux distro from the Microsoft Store
- Install the [ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  - NOTE: You may find this in your distro's package manager, but ensure it is version 13.2.
  - Ubuntu has an outdated version in its repositories. If using Ubuntu, download it manually from the link above
- Follow the rest of the instructions in [Getting Started](#-getting-started)

### Flashing

After plugging in J-Link, it will likely show up as unknown.

1. Install Zadig: https://zadig.akeo.ie/
2. From Zadig, select J-Link as the device and WinUSB as the driver
3. click Install Driver

If using WSL or Docker with a WSL backend (you probably are), you need to tell Windows to share J-Link with WSL.

- From WSL, install linux-tools-generic
  - on Ubuntu: `sudo apt install linux-tools-generic`
- Install usbipd-win: https://github.com/dorssel/usbipd-win/releases
- Open command prompt/powershell with admin privileges and run `usbipd list`
- Make note of the entry called J-Link and run `usbipd bind --busid <busid>`
- Next, run `usbipd attach --wsl --busid <busid>`
- You can now follow the flashing instructions in [Getting Started](#getting-started)

## Documentation

Run `cargo doc --open` to build and open the documentation. Most documentation for this repo is contained in the `common-arm` crate.

Documentation is also automatically built and deployed to https://hydra-docs.uorocketry.ca/common_arm

## Project Structure

The project is structured in a way to allow reuse of the code across various boards and other projects.

- `boards`: Folder containing each individual board's binary crates. Any code in those crates should only contain logic specific to the board.
- `debug`: Useful files for debugging, such as GDB and OpenOCD configuration.
- `examples`: Example projects that can be used to quickly start a new board. Simply copy and paste one of those creates to the `boards` folder, and rename as needed.
- `libraries`:
  - `common-arm`: Common code that depends on embedded-specific logic or crates.

## License

This project is licensed under GPLv3-only.

We please ask for any derivatives of this work to be kept open-source, even if such derivative is only for internal use.
