idf_component_register(
    SRCS "src/ota_ble.cpp" "src/ota_xform.cpp"
    INCLUDE_DIRS "src"
    # tamp and esp_delta_ota back the wire transforms and are declared in idf_component.yml, so the
    # component manager fetches them for every IDF consumer. The sources still guard both with
    # __has_include: the PlatformIO/Arduino consumer builds from library.json, which does not pull
    # them, and there the transforms simply report themselves unavailable over the wire.
    # Both tamp spellings are listed because a project may carry either: "brianpugh__tamp" is the
    # registry component idf_component.yml asks for, "tamp" a vendored copy of the same sources
    # under components/. A REQUIRES entry that names a component the build does not have is dropped
    # rather than fatal (verified: this component built with brianpugh__tamp listed and only tamp
    # present), so naming both resolves whichever exists instead of relying on some other
    # component's requires to leak the include directory in.
    REQUIRES app_update esp_partition mbedtls esp_system
             tamp brianpugh__tamp espressif__esp_delta_ota
)
