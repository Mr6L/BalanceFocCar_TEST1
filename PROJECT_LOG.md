# Balance FOC Car 项目调试日志

> 创建时间：2026-07-09  
> 用途：记录项目结构、配置变更与调试状态，便于在新目录下继续开发。

---

## 1. 项目概述

这是一个基于 **ESP32-S3 + SimpleFOC** 的自平衡小车（Balance FOC Car）固件，使用 **PlatformIO / Arduino** 框架开发。

- **目标芯片**：ESP32-S3-N16R8（16MB Flash + 8MB PSRAM）
- **开发框架**：Arduino
- **核心依赖库**：
  - `SimpleFOC@2.3.3`：无刷电机 FOC 控制
  - `OneButton@^2.5.0`：按键处理
  - `Adafruit NeoPixel@^1.12.2`：WS2812 RGB 灯带
- **主入口**：`src/main.cpp`

---

## 2. 项目结构

```
BalanceFocCar_TEST1/
├── platformio.ini          # 平台/板子/分区/PSRAM 配置
├── src/main.cpp            # 主入口，创建 4 个 FreeRTOS 任务
├── lib/
│   ├── UserConfig/         # UserConfig.h：所有宏配置（PID、引脚、WiFi、安全阈值）
│   ├── SuperCar/           # 平衡控制核心：FOC 电机、MPU6050、PID、状态机
│   ├── APP/                # WiFi AP/STA + WebServer + 远程 HTTP 轮询 + 行车控制
│   ├── BluetoothCarControl/# 小智 BLE 串口协议控制
│   ├── Button/             # 电源键、电池 ADC、低电量报警
│   ├── RGB/                # 底盘灯、前后灯 WS2812
│   ├── Buzzer/             # 蜂鸣器音效
│   ├── MPU6050/            # MPU6050 I2C 驱动
│   └── PID/                # 直立环 PID 实现
└── test/                   # 独立功能测试：编码器、MPU6050、WiFi、FOC 单/双电机
```

---

## 3. 主要任务与架构

`main.cpp` 在 `setup()` 中启动以下任务：

| 任务 | 文件 | 核心职责 |
|---|---|---|
| Button Event | `ButtonAndBattery.cpp` | 长按关机、单击关灯 |
| Battery Voltage Check | `ButtonAndBattery.cpp` | 每 500ms 采样电池电压，低电量报警 |
| XiaoZhi BLE | `BluetoothCarControl.cpp` | 蓝牙命令解析：车/前进/距离/转弯/RGB |
| App Server / Brake / Remote | `APP.cpp` | WiFi、HTTP 服务、驾驶斜坡、远程轮询 |
| BalanceCar `running()` | `SuperCar.cpp` | `loop()` 中高频执行 FOC + 姿态 + 平衡控制 |

### 平衡控制链路

1. `motor_A.loopFOC()` / `motor_B.loopFOC()` 更新电机
2. `mpu6050.update()` 读取角度、角速度、加速度模
3. **速度环** `Velocity_PID`：目标速度 → 目标倾角
4. **直立环** `UprightPID`：目标倾角 + 当前角度/角速度 → 电机速度
5. 叠加转向速度后 `motor.move()`

---

## 4. 关键配置变更

### 4.1 platformio.ini 已调整为 ESP32-S3-N16R8

```ini
[platformio]
default_envs = esp32-s3-n16r8

[env:esp32-s3-n16r8]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

upload_speed = 921600
monitor_speed = 115200

; ESP32-S3-N16R8: 16MB Flash + 8MB PSRAM
board_build.flash_size = 16MB
board_upload.flash_size = 16MB
board_build.partitions = default_16MB.csv
board_build.arduino.memory_type = qio_opi

build_flags =
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1

lib_deps =
    mathertel/OneButton@^2.5.0
    askuric/Simple FOC@2.3.3
    adafruit/Adafruit NeoPixel@^1.12.2
```

### 4.2 项目路径迁移

原路径 `D:/实习/咸鱼资料/BalanceFocCar_TEST1/BalanceFocCar_TEST1` 含中文，会导致 GCC 链接器找不到 `.map` 文件：

```
cannot open map file D:/ʵϰ/��������/.../firmware.map: No such file or directory
```

**已复制到纯英文路径**：

```
D:/BalanceFocCar_TEST1
```

后续开发请在此目录下进行。

---

## 5. 当前状态

- [x] `platformio.ini` 已配置为 ESP32-S3-N16R8
- [x] 编译通过（`pio run`）
- [x] 固件生成成功
  - `firmware.bin`：1.6 MB
  - `partitions.bin`：3 KB（使用 `default_16MB.csv`）
  - `bootloader.bin`：15 KB
- [ ] 实际硬件烧录验证
- [ ] PSRAM 上电检测验证
- [ ] 电机/编码器/MPU6050 硬件联调

---

## 6. 下一步操作建议

### 6.1 打开新目录

在 VS Code 中打开：

```
D:/BalanceFocCar_TEST1
```

### 6.2 烧录到硬件

连接 ESP32-S3 开发板后执行：

```bash
pio run --target upload
```

或在左侧 PlatformIO 面板点击 **Upload**。

### 6.3 查看串口输出

```bash
pio device monitor
```

波特率：115200

### 6.4 验证 PSRAM 是否启用

在 `setup()` 中临时加入：

```cpp
Serial.printf("Flash size: %d bytes\n", ESP.getFlashChipSize());
Serial.printf("PSRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
```

预期结果：

- `Flash size: 16777216`（16 MB）
- `PSRAM free size` 有值，约 8MB 左右

若出现 `PSRAM ID read error`，需回头检查 `memory_type` 是否需要调整。

---

## 7. 常用调试命令

```bash
# 编译
pio run

# 清理后编译
pio run --target clean
pio run

# 烧录
pio run --target upload

# 串口监视
pio device monitor

# 查看端口
pio device list
```

---

## 8. 注意事项

1. **路径不要含中文**：PlatformIO 的 GCC 工具链对中文路径支持不佳。
2. **default_16MB.csv 的 APP 分区为 6.25MB**：目前固件 1.6MB，足够；后续代码或文件系统变大时需定制分区表。
3. **开发板默认名为 `esp32-s3-devkitc-1-N8`**：PlatformIO 没有专门的 N16R8 板型，但配置参数已覆盖为 16MB Flash + OPI PSRAM。
4. **当前电流检测已禁用**：`SuperCar.cpp` 注释说明在 Arduino-ESP32 2.x 下 SimpleFOC 的 MCPWM 电流检测 ISR 可能在 WiFi/NVS 访问时崩溃。

---

## 9. 待确认/待调试项

- [ ] 上电后 `esp_reset_reason()` 输出是否正常
- [ ] FOC 初始化结果 `initFOC()` 是否返回 1
- [ ] MPU6050 是否能正常校准并输出稳定角度
- [ ] AS5600 编码器 A/B 是否能独立读取角度和速度
- [ ] 电机转向是否与 `MOTOR_A_BALANCE_SIGN` / `MOTOR_B_BALANCE_SIGN` 设定一致
- [ ] 自平衡是否能正常启动并进入 `CAR_STATE_BALANCING`
- [ ] WiFi AP / WebServer / BLE 是否能正常连接

---

## 10. 参考资料

- [PlatformIO ESP32 文档](https://docs.platformio.org/page/boards/espressif32/esp32-s3-devkitc-1.html)
- [SimpleFOC 文档](https://docs.simplefoc.com/)
- 项目原有测试说明：`test/README.md`
