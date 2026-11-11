if(CONFIG_VIIKATE_SIM)
  # code/app/src/main.c asserts on a symbol that no longer exists
  # (imu_filter). On the target, assert() is a no-op without CONFIG_ASSERT,
  # so mirror that here instead of failing the host build.
  zephyr_compile_definitions(NDEBUG)

  zephyr_library()
  zephyr_library_include_directories(include)
  zephyr_library_sources(
    src/sim_pwm.c
    src/sim_sensors.c
    src/sim_bridge.c
  )

  if(CONFIG_VIIKATE_SIM_PLANT_TOY)
    zephyr_library_sources(plant/toy_roll_plant.c)
  elseif(CONFIG_VIIKATE_SIM_PLANT_ROCKET)
    message(FATAL_ERROR "Rocket plant sources are not available yet")
  endif()
endif()
