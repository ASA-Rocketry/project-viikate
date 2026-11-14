cmake_minimum_required(VERSION 3.20.0)

# logger.conf is shared with the HIL log test firmware.
set(CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/prj.conf ${CMAKE_CURRENT_LIST_DIR}/logger.conf)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_app)

include(${CMAKE_CURRENT_SOURCE_DIR}/sources.cmake)
