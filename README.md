# IMU-AHRS

IMU-AHRS 是一个基于 ESP32-C3 和 MPU6500/MPU9250 的姿态采集项目。设备端读取 IMU 数据并计算四元数姿态，通过蓝牙低功耗（BLE）发送到电脑；PC 程序可实时显示姿态，并提供 IMU 与 PC 时钟对齐能力，便于后续与相机或其他传感器数据一起使用。

## 项目结构

```text
MPU6500 / MPU9250 -> ESP32-C3 固件 -> BLE -> PC 查看器
```

- `firmware/`：ESP32-C3 固件，负责 IMU 采样、姿态解算和 BLE 通信。
- `pc/`：PC 端 Python 程序，负责连接设备、显示姿态和记录时钟同步状态。
- `firmware/components/`：项目使用的算法、IMU 驱动和 BLE 组件。

默认硬件连接：I2C SDA 为 GPIO6，SCL 为 GPIO7，IMU 的 `INT` 接 GPIO10（3.3 V 高电平、上升沿触发）。如硬件接线不同，可在固件配置中调整。

## 所需环境

- ESP32-C3 开发板与 MPU6500 或 MPU9250 模块
- 支持 BLE 的 Windows 或 Ubuntu 电脑
- Python 3.11 或更高版本
- ESP-IDF 6.0.2（用于编译和烧录 ESP32 固件）

Windows 需要安装开发板对应的 USB 串口驱动；Ubuntu 需要可用的蓝牙适配器和 BlueZ 蓝牙服务。

## 使用方法

### 1. 克隆项目

```bash
git clone https://github.com/hu366/IMU_AHRS.git
cd IMU_AHRS
```

### 2. 编译并烧录固件

先按 ESP-IDF 官方说明安装并初始化 ESP-IDF 6.0.2，然后在已加载 ESP-IDF 环境的终端运行：

```bash
cd firmware
idf.py set-target esp32c3
idf.py build
idf.py -p <串口> flash monitor
```

Windows 的串口通常类似 `COM10`；Ubuntu 通常类似 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。烧录完成后，设备会以 `IMU-AHRS` 名称开始 BLE 广播。

### 3. 安装并运行 PC 程序

在仓库根目录执行以下命令。

Windows PowerShell：

```powershell
cd pc
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[sync]"
python -m imu_viewer --name IMU-AHRS
```

Ubuntu：

```bash
cd pc
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[sync]"
python -m imu_viewer --name IMU-AHRS
```

程序会扫描并连接 `IMU-AHRS`，随后显示实时姿态。只检查 PC 与设备的时钟同步状态时，可运行：

```bash
python -m imu_viewer --name IMU-AHRS --quiet --sync-report
```

首次排查 PC 程序时，也可以不连接硬件：

```bash
python -m imu_viewer --demo
```

## 验证

PC 端测试不需要连接 ESP32 或蓝牙设备。在仓库根目录执行：

```bash
pytest pc/tests
```

## 仓库内容

仓库只保存源代码、配置和必要的第三方许可证/来源声明。Python 虚拟环境、通过 `pip` 安装的库、ESP-IDF 构建产物、串口日志和本地文档不会上传；克隆后按上面的命令重新安装即可。
