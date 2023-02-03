To generate the SBG bindings:
1. Install `bindgen`:
```
cargo install bindgen-cli
```
2. Make sure the GCC ARM cross-compiler (`gcc-arm-none-eabi`) is installed on your computer. It can also be downloaded here: https://developer.arm.com/downloads/-/gnu-rm
3. Run `bindgen`, replacing the `--sysroot` with the correct path
```
bindgen src/sbgEComLib.h -o bindings.rs --use-core -- -I./src -I./common --sysroot=/path/to/arm-none-eabi/ -target thumbv7em-none-eabihf -mcpu=cortex-m4 -mthumb -mfloat-abi=hard
```
