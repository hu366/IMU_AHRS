idf_component_register(SRCS "main.c"
                            "imu_task.c"
                            "preprocess.c"
                            "ahrs.c"
                            "pose_protocol.c"
                            "time_sync_protocol.c"
                            "imu_trigger.c"
                            "ble_app.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_timer esp_driver_gpio mpu9250 madgwick vqf ble_uart bt
                    # Keep the concrete BLE and protocol objects linked from
                    # this component even across static-library boundaries.
                    WHOLE_ARCHIVE)
