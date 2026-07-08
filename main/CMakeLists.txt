idf_build_get_property(component_targets __COMPONENT_TARGETS)

if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif ("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
elseif(DEFINED ENV{ESP_DL_PATH})
    set(espdl_dir $ENV{ESP_DL_PATH})
else()
    set(espdl_dir "D:/esp-dl/esp-dl")
endif()

set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)


idf_component_register(
    SRCS
        "main.c"
        "ads1298r.c"
        "i2c_sensors.c"
        "pipeline.c"
        "sd_logger.c"
        "serial_stream.c"
        "ecg_inference.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp-dl
        fatfs
        sdmmc
        esp_timer
        esp_driver_spi
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_uart
        esp_driver_usb_serial_jtag
    WHOLE_ARCHIVE
)


target_add_aligned_binary_data(${COMPONENT_LIB} "models/model.espdl" BINARY)