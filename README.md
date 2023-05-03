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
4. Flash: `cargo run --bin main`
5. Run tests: `cargo test -p common-arm-test`

For more detailed instructions on flashing, debugging, and more, please see [the wiki](https://avwiki.uorocketry.ca/en/Avionics/HYDRA/Software).

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