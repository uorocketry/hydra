# HYDRA &emsp; [![Build Status]][actions] [![docs-badge]][docs]
*HYper Dynamic Rocketry Avionics*

[Build Status]: https://github.com/uorocketry/hydra/actions/workflows/build.yml/badge.svg
[actions]: https://github.com/uorocketry/hydra/actions?query=branch%3Amaster
[docs-badge]: https://img.shields.io/github/actions/workflow/status/uorocketry/hydra/docs.yml?label=docs
[docs]: http://hydra-docs.uorocketry.ca/common_arm

**uORocketry's next-generation avionics system**

---

## Getting Started

1. Install Rust: https://www.rust-lang.org/tools/install
2. Build: `cargo build`
3. Install probe-run: `cargo install --git https://github.com/uorocketry/probe-run`
    - `probe-run` currently requires a patch to flash our chip, so please use the above version while the patch is upstreamed
4. Install cargo-make: `cargo install cargo-make`
4. Flash: `cargo run --bin main`
5. Run tests: `cargo make test-host` or `cargo make test-device`

For more detailed instructions on flashing, debugging, and more, please see [the wiki](https://avwiki.uorocketry.ca/en/Avionics/HYDRA/Software).

## Windows Users

### Setting up with Docker
1. Install Docker: https://docs.docker.com/desktop/install/windows-install/
2. Install VS Code: https://code.visualstudio.com/download
3. From VS Code, install the "Dev Containers" extension
4. press `ctrl` + `shift` + `p`, and search for `Dev Containers: Open Folder in Container`

### Setting up with WSL
1. Enable WSL: https://learn.microsoft.com/en-us/windows/wsl/install
2. Install a linux distro from the Microsoft Store
4. Install ARM GNU Toolchain: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
  - NOTE: You may find this in your distro's package manager, but ensure it is version 13.2.
  - Ubuntu has an outdated version in its repositories. If using Ubuntu, download it manually from the link above
3. Follow the rest of the instructions in [Getting Started](#-getting-started)

### Flashing
After plugging in J-Link, it will likely show up as unknown.
1. Install Zadig: https://zadig.akeo.ie/
2. From Zadig, select J-Link as the device and WinUSB as the driver
3. click Install Driver

If using WSL or Docker with a WSL backend (you probably are), you need to tell Windows to share J-Link with WSL.
1. From WSL, install linux-tools-generic
  1. on Ubuntu: `sudo apt install linux-tools-generic`
2. Install usbipd-win: https://github.com/dorssel/usbipd-win/releases
3. Open command prompt/powershell with admin privileges and run `usbipd list`
4. Make note of the entry called J-Link and run `usbipd bind --busid <busid>`
5. Next, run `usbipd attach --wsl --busid <busid>`
6. You can now follow the flashing instructions in [Getting Started](#-getting-started)

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
