# The following variables contains the files used by the different stages of the build process.
set(water_meter_counter_default_default_XC8_FILE_TYPE_assemble)
set_source_files_properties(${water_meter_counter_default_default_XC8_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${water_meter_counter_default_default_XC8_FILE_TYPE_assemble})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(water_meter_counter_default_default_XC8_FILE_TYPE_assemblePreprocess)
set_source_files_properties(${water_meter_counter_default_default_XC8_FILE_TYPE_assemblePreprocess} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${water_meter_counter_default_default_XC8_FILE_TYPE_assemblePreprocess})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(water_meter_counter_default_default_XC8_FILE_TYPE_compile
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/comparator/src/cmp1.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/comparator/src/cmp2.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/examples/UART_Example/uart_example.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/peripheral/src/cvref.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/clock.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/config_bits.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/interrupt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/pins.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/system.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/timer/src/tmr0.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/uart/src/eusart1.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../rotation_decoder.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../water_meter.c")
set_source_files_properties(${water_meter_counter_default_default_XC8_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(water_meter_counter_default_default_XC8_FILE_TYPE_link)
set(water_meter_counter_default_image_name "default.elf")
set(water_meter_counter_default_image_base_name "default")

# The output directory of the final image.
set(water_meter_counter_default_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/water-meter-counter")

# The full path to the final image.
set(water_meter_counter_default_full_path_to_image ${water_meter_counter_default_output_dir}/${water_meter_counter_default_image_name})

# Potential output file extensions
set(output_extensions
    .hex
    .hxl
    .mum
    .o
    .sdb
    .sym
    .cmf)
list(TRANSFORM output_extensions PREPEND "${water_meter_counter_default_output_dir}/${water_meter_counter_default_image_base_name}")
