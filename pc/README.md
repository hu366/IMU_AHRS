# IMU-AHRS PC 查看器（Agent C）

把固件发出的 `Q,w,x,y,z\n` 解析成姿态，在 PC 上用 VPython 以约 60 Hz 画一个 3D 方块。

第一阶段完全离线：`--demo` 和 `pytest` **不需要 ESP32、不需要 MPU9250、不需要 BLE**。

## 环境

- Python **3.11+**（开发机实测 3.13）
- Windows 上连真机还需要蓝牙适配器与系统蓝牙权限

```powershell
cd pc
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
python -m pip install -r requirements.txt
```

依赖见 `requirements.txt`：`bleak>=0.22`、`vpython>=7.6.5`、`pytest>=8.0`。

也可以在本目录执行 `pip install -e .`（渲染还要 vpython，测试只要 pytest）。

## 运行

在 `pc/` 目录、venv 已激活时：

```powershell
python -m imu_viewer --help
python -m imu_viewer --demo
```

`--demo` 用定时器喂假四元数：每 8 秒绕手背系 X、Y、Z 各转一整圈。关窗口、按 `q` 或 Ctrl+C 退出。

连真机（需要 Agent B 的 `FW-06` 已广播 `IMU-AHRS`）：

```powershell
python -m imu_viewer
python -m imu_viewer --name IMU-AHRS
```

BLE 回调只 `feed()` 字节并更新最新姿态；VPython 只在渲染线程里读拷贝后的 `PoseState`。默认会把解析成功的四元数打印到控制台（`--quiet` 关闭打印）。

## 测试

不需要 BLE、不需要 ESP32。在仓库根目录或 `pc/` 下均可：

```powershell
# 仓库根目录
pytest pc/tests

# 或在 pc/
pytest tests
```

覆盖：单帧 / 粘包 / 分片、空行与非法帧、X/Y/Z 90° 朝向、超时用 `time.monotonic()`、假 notify 回调。

## 协议与 BLE 身份（冻结契约）

| 项 | 约定 |
| --- | --- |
| 帧 | `Q,w,x,y,z\n`（UTF-8）。偶发 `\r\n` 会剥掉 `\r`，固件不必发 `\r\n` |
| 分片 | 按字节缓存，遇到 `\n` 再解析 |
| 非法帧 | 空行、字段数不对、非数字、NaN/Inf、模长 `< 1e-6` 或 `> 2`：丢弃并计数 |
| 扫描 | 名称 `IMU-AHRS`，或 NUS UUID `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| TX Notify | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |
| RX | `6e400002-...` MVP 可连但不写 |
| 加密 | PC **不绕过配对**。若卡住，视为阻塞：Agent B 未关 `encrypted=false` |

标准向量（与 Agent B 同一组）：

```text
Q,1.0000,0.0000,0.0000,0.0000
Q,0.9981,0.0123,-0.0310,0.0512
```

坐标：固件已变到手背系 `h`（`X` 手腕→手指，`Z` 垂直手背向外，`Y` 右手定则）。PC **不做第二套轴交换**，只把 Hamilton 四元数转成 VPython `axis/up`。

## 阻塞（不要把 demo 当成端到端）

| 项 | 状态 |
| --- | --- |
| `--demo` / `pytest pc/tests` | PC 侧可独立验收 |
| `INT-01` 假四元数 BLE | **阻塞：等 Agent B `FW-06`**（广播名、NUS Notify、关加密） |
| `INT-03` 真姿态跟随 | **阻塞：等 INT-01 以及 Agent A `INT-02` / `FW-07`** |

真机连不上时先看日志：扫不到设备、解析 `drop` 上升、还是提示 pairing/encryption。不要在 PC 里补丁轴映射。

## 已知限制（MVP）

- 无磁力计，yaw 会漂
- 无陀螺仪零偏估计
- 无手背参考姿态标定；单位四元数即初始姿态
- 无 SLERP、无录制回放、无多设备
