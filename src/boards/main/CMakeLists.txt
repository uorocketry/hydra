#project(main)

add_executable(main main.cpp syscalls.c)
set_target_properties(main PROPERTIES OUTPUT_NAME "main.elf")

target_link_libraries(main PUBLIC shared)

add_subdirectory(hal)

add_custom_command(TARGET main POST_BUILD
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/bin
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:main> ${CMAKE_BINARY_DIR}/bin/main.bin
        COMMENT "Building ${BIN_FILE}")
