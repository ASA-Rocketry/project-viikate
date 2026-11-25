cmake_minimum_required(VERSION 3.20.0)

# The logger's settings live in logger.conf.
set(CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/prj.conf ${CMAKE_CURRENT_LIST_DIR}/logger.conf)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app)

include(${CMAKE_CURRENT_SOURCE_DIR}/sources.cmake)
