idf_component_register(
    SRCS "src/ota_ble.cpp"
    INCLUDE_DIRS "src"
    REQUIRES app_update esp_partition mbedtls esp_system
)
