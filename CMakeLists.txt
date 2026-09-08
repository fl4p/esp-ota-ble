idf_component_register(
    SRCS "src/ota_ble.cpp" "src/ota_xform.cpp"
    INCLUDE_DIRS "src"
    # tamp and esp_delta_ota back the wire transforms and are declared in idf_component.yml, so the
    # component manager fetches them for every IDF consumer. The sources still guard both with
    # __has_include: the PlatformIO/Arduino consumer builds from library.json, which does not pull
    # them, and there the transforms simply report themselves unavailable over the wire.
    REQUIRES app_update esp_partition mbedtls esp_system
             brianpugh__tamp espressif__esp_delta_ota
)
