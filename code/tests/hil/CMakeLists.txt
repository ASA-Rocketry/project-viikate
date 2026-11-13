# Hardware-in-the-loop boot test: builds the unmodified flight application
# (code/app) with its own prj.conf, overlay and devicetree bindings, so the
# image under test is the one that would fly.
cmake_minimum_required(VERSION 3.20.0)

get_filename_component(VIIKATE_APP ${CMAKE_CURRENT_LIST_DIR}/../../app ABSOLUTE)

set(CONF_FILE ${VIIKATE_APP}/prj.conf)
set(DTC_OVERLAY_FILE ${VIIKATE_APP}/app.overlay)
list(APPEND DTS_ROOT ${VIIKATE_APP})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(viikate_hil)

include(${VIIKATE_APP}/sources.cmake)
