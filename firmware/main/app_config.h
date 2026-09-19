#pragma once

/* --- IMU / I2C（Agent A 可按硬件改数值） --- */
#define APP_I2C_SDA_GPIO          6
#define APP_I2C_SCL_GPIO          7
#define APP_I2C_FREQ_HZ           400000
#define APP_MPU9250_ADDR          0x68      /* AD0=GND；若为 0x69 只改这里 */
#define APP_SAMPLE_HZ             100
#define APP_ACCEL_FS_G            2         /* ±2 g */
#define APP_GYRO_FS_DPS           250       /* ±250 dps；换算成 rad/s 后再进 AHRS */
#define APP_MADGWICK_BETA         0.1f      /* 仅 APP_AHRS_ALGO=MADGWICK 时使用 */

#define APP_AHRS_ALGO_MADGWICK    0
#define APP_AHRS_ALGO_VQF         1
#define APP_AHRS_ALGO             APP_AHRS_ALGO_VQF   /* 默认 VQF；对照时改回 0 */

#define APP_VQF_TAU_ACC           3.0f    /* 官方默认量级，先不要调 */
#define APP_VQF_MOTION_BIAS       1
#define APP_VQF_REST_BIAS         1
/* VQF getQuat6D 与内部 quatRotate 把传感器矢量转到地球系，与现网 Madgwick/PC
   body→world 一致。若验收时方块相对 Madgwick 时期反向，改成 1，不要改 PC 轴映射。 */
#define APP_VQF_CONJUGATE_OUTPUT  0
#define APP_GRAVITY_MPS2          9.80665f  /* ahrs 把 g 转成 m/s² 再送 VQF；预处理仍是 g */
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

/* 启动陀螺仪零偏（3.1）：每次上电静止均值，手背系 rad/s，不写 NVS。
   MAX_STILL_* 用于拒绝运动窗口：典型静止 bias 约 1e-3～1e-1 rad/s，明显大于此则重试。 */
#define APP_GYRO_BIAS_S                 3.0f
#define APP_GYRO_BIAS_MIN_SAMPLES       200
#define APP_GYRO_BIAS_MAX_STILL_RAD     0.35f   /* any-axis mean |gyro|, ~20 dps */
#define APP_GYRO_BIAS_MAX_STILL_STD_RAD 0.10f   /* any-axis std, ~5.7 dps */

/* [3.3] UART still CSV for PC Allan. Off by default — not in the AHRS loop.
   Set to 1, flash, then open the Python monitor (it writes still.log):
     python -m analysis.capture_still --port COMx --analyze
   Raise S (e.g. 300) if you need bias-instability tau; 60 s is enough for white noise.
   Dump is axis-mapped, before 3.1 bias subtract. */
#define APP_LOG_STILL_CSV           0
#define APP_LOG_STILL_CSV_S         60.0f

/* [4.2] UART AHRS CSV for PC offline VQF vs Madgwick. Off by default.
   Set to 1, flash, then (do not also run idf.py monitor):
     python -m analysis.capture_ahrs --port COMx
   Dump starts after 3.1 bias + ahrs_init. Rows are the ahrs_update input
   (hand frame, bias subtracted, accel g) plus the device quaternion.
   100 Hz x 11 columns is tight at 115200: stats logs are paused during dump.
   If rows drop, raise CONFIG_ESP_CONSOLE_UART_BAUDRATE to 921600 for the
   recording firmware only, then restore. Do not decimate (VQF uses fixed Ts). */
#define APP_LOG_AHRS_CSV            0
#define APP_LOG_AHRS_CSV_S          120.0f

/* --- BLE（照抄，禁止改 UUID / 名字）Agent B 使用 --- */
#define APP_BLE_DEVICE_NAME       "IMU-AHRS"
#define APP_BLE_ENCRYPTED         0

/* Fake quaternion Notify for INT-01 without MPU. Keep 1 until WHO_AM_I
   succeeds; imu_task only calls ble_send_line after AHRS, so dual TX
   cannot happen while I2C probe fails. Set to 0 when IMU Notify is live. */
#define APP_BLE_FAKE_QUAT         0
#define APP_BLE_FAKE_HZ           50
