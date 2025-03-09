file(GLOB_RECURSE srcs "main.cpp"
        "i2c.h"
        "i2c.cpp"
        "uint3.h"
        "devices/*.h"
        "devices/*.cpp"
        "ble/ble.h"
        "ble/ble.cpp")

idf_component_register(SRCS "${srcs}"
        PRIV_REQUIRES driver esp_driver_i2c bt nvs_flash)
