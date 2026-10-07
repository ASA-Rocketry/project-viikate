cmake_minimum_required(VERSION 3.20.0)
project(viikate_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

add_compile_options(-Wall -Wextra -Werror)

set(TEST_ARGS "" CACHE STRING "arguments forwarded to every test executable")
separate_arguments(TEST_ARGS_LIST UNIX_COMMAND "${TEST_ARGS}")

add_library(utest INTERFACE)
target_include_directories(utest INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor
)

set(guidance_include ${CMAKE_CURRENT_SOURCE_DIR}/../code/app/include/math)

file(GLOB test_sources CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/test_*.c
)

foreach(source IN LISTS test_sources)
    get_filename_component(name ${source} NAME_WE)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE ${guidance_include})
    target_link_libraries(${name} PRIVATE utest m)
    add_test(NAME ${name} COMMAND ${name} ${TEST_ARGS_LIST})
endforeach()
