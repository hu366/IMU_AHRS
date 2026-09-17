# madgwick

- 来源：Sebastian O. H. Madgwick, *An efficient orientation filter for inertial and inertial/magnetic sensor arrays*, 2010；官方 C 实现 `MadgwickAHRS.c` 的 **IMU 更新**（无磁力计），x-io Technologies。
  参考：http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms
- 版本：2011-02-19 官方 IMU 更新公式（`MadgwickAHRSupdateIMU`）。本仓库不整文件拷贝原实现，改为实例 + 真实 `dt` 的薄封装。
- 许可证：原 `MadgwickAHRS.c` 为作者/x-io 发布的开源算法实现。本封装仅保留 6DoF IMU 步进，供 `ahrs.c` 调用。
- 改了哪些文件：
  - 用结构体保存 `q0..q3` 与 `beta`，不再用全局 `sampleFreq`。
  - 积分使用调用方传入的 `dt`，禁止写死 0.01 s。
  - `invSqrt` 改为 `1/sqrtf`，避免 fast-inv-sqrt 在 RISC-V 上的严格别名问题。
  - 不包含磁力计 / MARG 更新。
