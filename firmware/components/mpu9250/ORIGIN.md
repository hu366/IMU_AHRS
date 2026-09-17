# mpu9250

- 来源：自写封装。I2C 主机路径照 ESP-IDF v6.0.2 `examples/peripherals/i2c/i2c_basic`（该例程本身就是 MPU9250 `WHO_AM_I`）。寄存器与量程换算来自 `doc/hardware/mpu9250/datasheets/RM-MPU-9250A-00.pdf`。
- 未采用 `hiruna/esp-idf-mpu-9250`：面向旧 `driver/i2c.h`，与 IDF v6.0.2 的 `i2c_master` 不兼容。任务书要求该路径编不过则立刻走 `i2c_master` 兜底。
- 未采用 `truita/mpu9250`：同样为避开旧 I2C API / 组件注册表依赖。
- 版本：无第三方 commit；对齐 ESP-IDF v6.0.2。
- 许可证：本组件代码按工程使用（与 ESP-IDF 示例 Unlicense/CC0 的 I2C 调用方式一致）。寄存器含义来自 InvenSense 公开手册。
- 改了哪些文件：全新 `mpu9250.c` / `include/mpu9250.h` / `CMakeLists.txt`，无拷贝第三方驱动源码。
