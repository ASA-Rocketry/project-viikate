# Application sources, shared by the firmware build (code/app) and the
# hardware-in-the-loop test build (code/tests/hil).
set(VIIKATE_APP_DIR ${CMAKE_CURRENT_LIST_DIR})

target_include_directories(app PRIVATE ${VIIKATE_APP_DIR}/include)

target_sources(app PRIVATE
    ${VIIKATE_APP_DIR}/src/main.c
    ${VIIKATE_APP_DIR}/src/servo.c
    ${VIIKATE_APP_DIR}/src/sensors.c
    ${VIIKATE_APP_DIR}/src/guidance/kalman.c
    ${VIIKATE_APP_DIR}/src/guidance/filters.c
    ${VIIKATE_APP_DIR}/src/guidance/pid.c
)
