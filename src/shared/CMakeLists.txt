file(GLOB SOURCES "*.cpp")

add_library(shared test.cpp)

target_include_directories(shared PUBLIC ./)

