cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

# Using ARM
# set(CMAKE_C_COMPILER arm-none-eabi-gcc)
# set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
# set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
# set(CMAKE_AR arm-none-eabi-ar)
# set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
# set(CMAKE_OBJDUMP arm-none-eabi-objdump)
# set(SIZE arm-none-eabi-size)

# Using XC32
set(CMAKE_C_COMPILER /opt/microchip/xc32/v4.10/bin/xc32-gcc)
set(CMAKE_CXX_COMPILER /opt/microchip/xc32/v4.10/bin/xc32-g++)
set(CMAKE_ASM_COMPILER  /opt/microchip/xc32/v4.10/bin/xc32-gcc)
set(CMAKE_AR /opt/microchip/xc32/v4.10/bin/xc32-ar)
set(CMAKE_OBJCOPY /opt/microchip/xc32/v4.10/bin/xc32-objcopy)
set(CMAKE_OBJDUMP /opt/microchip/xc32/v4.10/bin/xc32-objdump)
set(SIZE /opt/microchip/xc32/v4.10/bin/xc32-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(Hydra C CXX)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

add_compile_options(-mfloat-abi=softfp -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=softfp -mfpu=fpv4-sp-d16)

# Using ARM for the SAME51J18A
# add_compile_definitions(__SAME51J18A__)
# add_compile_options(-mcpu=cortex-m4 -mthumb)
# add_compile_options(-ffunction-sections -mlong-calls )

# link_directories(linking)
# add_link_options(-Wl,--start-group -lm -Wl,--end-group -mthumb)
# add_link_options(--specs=nano.specs -Wl,--gc-sections -mcpu=cortex-m4)
# add_link_options(-T same51j18a_flash.ld)

# Using XC32 for the SAME51J20A
add_compile_definitions(__SAME51J20A__)
add_compile_options(-mprocessor=ATSAME51J20A -mthumb)
add_compile_options(-ffunction-sections -mlong-calls )

link_directories(linking)
add_link_options(-Wl,--start-group -lm -Wl,--end-group -mthumb)
add_link_options(-Wl,--gc-sections -mprocessor=ATSAME51J20A)

# shared code
add_subdirectory(src/shared)

# include all boards here
add_subdirectory(src/boards/main)
add_subdirectory(src/boards/dev)
