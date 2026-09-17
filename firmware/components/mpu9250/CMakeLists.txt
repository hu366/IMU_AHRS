idf_component_register(SRCS "mpu9250.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_i2c freertos log)
