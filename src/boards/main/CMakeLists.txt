#project(main)

add_executable(main main.cpp syscalls.c)
set_target_properties(main PROPERTIES OUTPUT_NAME "main.hex")

target_link_libraries(main PUBLIC shared)

add_subdirectory(hal)

add_custom_command(TARGET main POST_BUILD
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signiture $<TARGET_FILE:main>
        COMMENT "Building ${BIN_FILE}")
