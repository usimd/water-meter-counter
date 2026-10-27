# SPDX-FileCopyrightText: 2025-2026 usimd
# SPDX-License-Identifier: Apache-2.0

# Included by the generated CMakeLists.txt after .generated/main.cmake.
#
# Builds a second image next to the production one, compiled with
# DEBUG_STREAM=1 (per-sample UART output, see main.c), from the same sources
# and with the same compiler/linker rules:
#
#   out/water-meter-counter/default.hex        production (DEBUG_STREAM 0)
#   out/water-meter-counter/default-debug.hex  debug output (DEBUG_STREAM 1)

set(_wmc_debug_name "${water_meter_counter_default_image_base_name}-debug")

add_library(water_meter_counter_debug_compile OBJECT
    ${water_meter_counter_default_default_XC8_FILE_TYPE_compile})
water_meter_counter_default_default_XC8_compile_rule(water_meter_counter_debug_compile)
target_compile_definitions(water_meter_counter_debug_compile PRIVATE "DEBUG_STREAM=1")

add_executable(water_meter_counter_debug_image
    $<TARGET_OBJECTS:water_meter_counter_debug_compile>)
set_target_properties(water_meter_counter_debug_image PROPERTIES
    OUTPUT_NAME "${_wmc_debug_name}"
    SUFFIX ".elf"
    RUNTIME_OUTPUT_DIRECTORY "${water_meter_counter_default_output_dir}")
water_meter_counter_default_link_rule(water_meter_counter_debug_image)
# The generated link rule writes mem.map/memoryfile.xml into the build
# directory; give the debug image its own map and link it after the
# production image so the two links never write the same files concurrently.
target_link_options(water_meter_counter_debug_image PRIVATE "-Wl,-Map=${_wmc_debug_name}.map")

get_property(_wmc_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
list(FILTER _wmc_targets INCLUDE REGEX "^water_meter_counter_default_image_")
if(_wmc_targets)
    add_dependencies(water_meter_counter_debug_image ${_wmc_targets})
endif()
