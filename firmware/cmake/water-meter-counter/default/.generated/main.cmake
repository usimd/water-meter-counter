include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(water_meter_counter_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(water_meter_counter_default_default_XC8_FILE_TYPE_assemble)
add_library(water_meter_counter_default_default_XC8_assemble OBJECT ${water_meter_counter_default_default_XC8_FILE_TYPE_assemble})
    water_meter_counter_default_default_XC8_assemble_rule(water_meter_counter_default_default_XC8_assemble)
    list(APPEND water_meter_counter_default_library_list "$<TARGET_OBJECTS:water_meter_counter_default_default_XC8_assemble>")

endif()

# Handle files with suffix S, for group default-XC8
if(water_meter_counter_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(water_meter_counter_default_default_XC8_assemblePreprocess OBJECT ${water_meter_counter_default_default_XC8_FILE_TYPE_assemblePreprocess})
    water_meter_counter_default_default_XC8_assemblePreprocess_rule(water_meter_counter_default_default_XC8_assemblePreprocess)
    list(APPEND water_meter_counter_default_library_list "$<TARGET_OBJECTS:water_meter_counter_default_default_XC8_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group default-XC8
if(water_meter_counter_default_default_XC8_FILE_TYPE_compile)
add_library(water_meter_counter_default_default_XC8_compile OBJECT ${water_meter_counter_default_default_XC8_FILE_TYPE_compile})
    water_meter_counter_default_default_XC8_compile_rule(water_meter_counter_default_default_XC8_compile)
    list(APPEND water_meter_counter_default_library_list "$<TARGET_OBJECTS:water_meter_counter_default_default_XC8_compile>")

endif()


# Main target for this project
add_executable(water_meter_counter_default_image_y7R7yoaU ${water_meter_counter_default_library_list})

set_target_properties(water_meter_counter_default_image_y7R7yoaU PROPERTIES
    OUTPUT_NAME "default"
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES "${output_extensions}"
    RUNTIME_OUTPUT_DIRECTORY "${water_meter_counter_default_output_dir}")
target_link_libraries(water_meter_counter_default_image_y7R7yoaU PRIVATE ${water_meter_counter_default_default_XC8_FILE_TYPE_link})

# Add the link options from the rule file.
water_meter_counter_default_link_rule( water_meter_counter_default_image_y7R7yoaU)


