# vqf

- 来源：[`DusKing1/vqf-c`](https://github.com/DusKing1/vqf-c)，纯 C 移植自官方 [`dlaidig/vqf`](https://github.com/dlaidig/vqf) full VQF。
- 锁定 commit：`2183c1dc4e4f0f66883cfc10021ebd550059b322`（仓库目前仅此 `init` 提交）。
- 许可证：MIT（见 `LICENSE`）。上游版权声明 `Copyright (c) 2024 Hugo Chiang`。
- 本仓库文件：
  - `vqf.c`、`include/vqf.h` 来自上游 `src/`
  - `LICENSE` 原样拷贝
  - `CMakeLists.txt` 为本工程 ESP-IDF 组件包装

## 算法用法（本工程）

- 单精度：`VQF_SINGLE_PRECISION`（头文件默认 + 组件 `compile_definitions`）。
- 6D：只调用 `updateGyr` + `updateAcc` + `getQuat6D`，**从不**调用 `updateMag`。
- `initVqf(gyrTs, accTs, magTs)` 的采样周期用标称 `1/APP_SAMPLE_HZ`。此 C 移植没有 per-sample `dt`；`magTs <= 0` 会回落到 `gyrTs`，磁力计仍靠不调用 `updateMag` 关闭。
- `magDistRejection` 在 `ahrs.c` 里 `setMagDistRejectionEnabled(false)` 关掉。
- 加速度单位：m/s²（`ahrs.c` 把预处理后的 g 乘 `9.80665`）。陀螺：rad/s。
- 四元数：`getQuat6D` 与内部 `quatRotate(q, acc)` 把传感器矢量转到地球系，与当前 PC `body → world` / 原 Madgwick 运动学一致。若真机方块反向，在 `ahrs.c` 把 `APP_VQF_CONJUGATE_OUTPUT` 置 1，不要改 PC 轴映射。

## 相对上游改了什么

上游不能直接编过 / 不能正确初始化，必须改：

1. **`initVqf` 补上 `init_params()`**  
   C++ 原版靠成员默认值（`tauAcc=3` 等）。此 C 移植写了 `init_params()` 却从未调用，`params` 全是 BSS 零；`filterCoeffs(tauAcc=0)` 会除零。每次 `initVqf` 先填官方默认参数再 `setup()`。
2. **`sin_fast` / `cos_fast` / `tan_fast` 未定义**  
   映射到 `sinf`/`cosf`/`tanf`（单精度）。
3. **补头文件** `stddef.h`（`size_t`）、`string.h`（`memcpy`）。
4. **单精度 libm**  
   `#define sqrt sqrtf` 等，避免 ESP32-C3 上软件 double。
5. **`resetState` 越界写**  
   删除 `restLastGyrLp[i]` 的 18 次循环（数组只有 3 个元素）。后面的 `motionBiasEst*` / rest LP 填充已覆盖原意。`restLastSquaredDeviations` 的 fill 长度 3→2。
6. **`setBiasEstimate` 的 `sizeof(bias)`**  
   形参退化成指针；改为 `sizeof(state.bias)`。本工程不调用该函数。
7. **头文件**  
   声明 `resetState`、`setTauAcc`、`setMotionBiasEstEnabled`、`setRestBiasEstEnabled`、`setMagDistRejectionEnabled`（实现本来就在 `vqf.c`）。`VQF_SINGLE_PRECISION` 改为若未定义再定义。
8. 删除未使用的 `TICK_INTERVAL`（上游写死 `4*0.000313`，与 100 Hz 无关）。

未改算法公式本身。无 ESP-IDF API。无 `basicvqf.*`（该仓库没有）。
