#project(dev)

add_executable(dev main.cpp syscalls.c)
set_target_properties(dev PROPERTIES OUTPUT_NAME "dev.hex")

target_link_libraries(dev PUBLIC shared)

add_subdirectory(hal)

add_custom_command(TARGET dev POST_BUILD
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/bin
	COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signiture $<TARGET_FILE:dev>
        COMMENT "Building ${BIN_FILE}")
