idf_component_register(
    SRCS "ble.c" "lora.c"
    INCLUDE_DIRS "."
    REQUIRES bt driver esp32)