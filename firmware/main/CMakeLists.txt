idf_component_register(SRCS "main.c"
                            "imu_task.c"
                            "preprocess.c"
                            "ahrs.c"
                            "pose_protocol.c"
                            "ble_app.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_timer mpu9250 madgwick ble_uart bt
                    # imu_task.c provides weak ble_app_start / ble_send_line /
                    # pose_protocol_encode. Without WHOLE_ARCHIVE, ld may pick
                    # those weaks from libmain.a and never extract ble_app.c.
                    WHOLE_ARCHIVE)
