cmake_minimum_required(VERSION 3.16)
project(droneflight_tether_plugin CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Parity with tether.py is bit-level, so don't let the compiler fuse
# `back_tension + 0.5*rho*...*v*v` into an FMA (one rounding instead of two).
# Defensive — it changes nothing on Apple clang today — but it must stay in step
# with tests/test_tether_plugin.py, which compiles the probe the same way.
add_compile_options(-ffp-contract=off)

# TetherPlugin.cc has never been compiled (no gz-sim8 on the Mac), so the first
# VM build is also its first review. Keep the warnings on.
add_compile_options(-Wall -Wextra)

# --------------------------------------------------------------------------- #
# tether_probe — the Mac-side parity fixture. No Gazebo, builds anywhere.
# --------------------------------------------------------------------------- #
add_executable(tether_probe tether_probe.cc)
target_include_directories(tether_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# --------------------------------------------------------------------------- #
# TetherPlugin — the Gazebo Harmonic system plugin. Ubuntu VM only.
# --------------------------------------------------------------------------- #
find_package(gz-plugin2 QUIET COMPONENTS register)
find_package(gz-sim8 QUIET)

if(gz-plugin2_FOUND AND gz-sim8_FOUND)
  set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

  add_library(TetherPlugin SHARED TetherPlugin.cc)
  target_include_directories(TetherPlugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(TetherPlugin
    PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

  message(STATUS "TetherPlugin: building against gz-sim${GZ_SIM_VER}")
  install(TARGETS TetherPlugin LIBRARY DESTINATION lib)
else()
  # Expected on the Mac: Gazebo Harmonic exists only on the Ubuntu VM. The
  # physics still gets compiled and tested here via tether_probe.
  message(STATUS
    "TetherPlugin: gz-sim8/gz-plugin2 not found — building tether_probe only. "
    "This is normal off the VM; the plugin itself needs Gazebo Harmonic.")
endif()
