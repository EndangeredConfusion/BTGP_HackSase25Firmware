#idf_component_register(SRCS "hello_world_main.c"
#                        REQUIRES driver
#                       PRIV_REQUIRES spi_flash
#        REQUIRES esp_wifi
#        PRIV_REQUIRES nvs_flash
#
#        INCLUDE_DIRS "")

idf_component_register(SRCS "hello_world_main.c"
                        REQUIRES driver
                       PRIV_REQUIRES spi_flash
        REQUIRES esp_wifi
        PRIV_REQUIRES nvs_flash
        REQUIRES esp_driver_i2c
        INCLUDE_DIRS "")
