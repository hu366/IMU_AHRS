#pragma once

/* --- IMU / I2C（Agent A 可按硬件改数值） --- */
#define APP_I2C_SDA_GPIO          6
#define APP_I2C_SCL_GPIO          7
#define APP_I2C_FREQ_HZ           400000
#define APP_MPU9250_ADDR          0x68      /* AD0=GND；若为 0x69 只改这里 */
#define APP_SAMPLE_HZ             100
#define APP_ACCEL_FS_G            2         /* ±2 g */
#define APP_GYRO_FS_DPS           250       /* ±250 dps；换算成 rad/s 后再进 Madgwick */
#define APP_MADGWICK_BETA         0.1f
#define APP_WHOAMI_MPU9250        0x71
#define APP_WHOAMI_MPU9255        0x73
#define APP_WHOAMI_MPU6500        0x70

/* 轴映射：传感器轴 -> 手背系。permute[i] 表示 h 系第 i 轴来自传感器的哪个轴(0=X,1=Y,2=Z)；
   sign[i] 为 +1/-1。只允许出现在 preprocess。默认先当安装已对齐，实测六面后再改。 */
#define APP_AXIS_PERMUTE_X        0
#define APP_AXIS_PERMUTE_Y        1
#define APP_AXIS_PERMUTE_Z        2
#define APP_AXIS_SIGN_X           1
#define APP_AXIS_SIGN_Y           1
#define APP_AXIS_SIGN_Z           1

/* --- BLE（照抄，禁止改 UUID / 名字）Agent B 使用 --- */
#define APP_BLE_DEVICE_NAME       "IMU-AHRS"
#define APP_BLE_ENCRYPTED         0

/* Fake quaternion Notify for INT-01 without MPU. Keep 1 until WHO_AM_I
   succeeds; imu_task only calls ble_send_line after AHRS, so dual TX
   cannot happen while I2C probe fails. Set to 0 when IMU Notify is live. */
#define APP_BLE_FAKE_QUAT         0
#define APP_BLE_FAKE_HZ           50
