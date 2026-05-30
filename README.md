# 管道检测机器人 — 嵌入式控制系统

> **Pipeline Inspection Robot — Embedded Control System**
>
> 香港理工大学机电工程系 2025/2026 毕业设计项目 (Final Year Project)
>
> 作者：ZENG Minyu &nbsp;|&nbsp; 代码最后更新：2026.04.22

---

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 系统架构](#2-系统架构)
- [3. 硬件规格](#3-硬件规格)
- [4. 引脚映射](#4-引脚映射)
- [5. 开发环境搭建](#5-开发环境搭建)
- [6. 编译与烧录](#6-编译与烧录)
- [7. 使用说明](#7-使用说明)
- [8. 软件架构](#8-软件架构)
- [9. 数据采集管线](#9-数据采集管线)
- [10. 关键技术问题与解决](#10-关键技术问题与解决)
- [11. 已知局限与改进方向](#11-已知局限与改进方向)
- [12. 项目文件结构](#12-项目文件结构)
- [13. 参考资料](#13-参考资料)

---

## 1. 项目概述

本项目为管道内壁检测机器人的嵌入式控制系统。机器人通过模块化分段设计进入管道内部，搭载传感器探头实现管道内壁的自动化扫描检测。

控制系统的核心功能：
- **四轴步进电机控制**：通过 CNC Shield v3 + A4988 驱动器分别控制扫描臂伸缩、360° 旋转扫描和整机行进
- **PS2 无线遥控**：16 键手柄实现对电机的选择、点动、连续运转和自动化序列触发
- **自动化检测序列**：预编程的运动轨迹，一键启动无人值守的管道扫描
- **上位机数据采集**：通过 USB 串口将运动数据实时传输至 PC，转换为柱坐标轨迹
- **安全保护**：继电器控制 12V 动力电源，双电池隔离供电，软件多级保护

---

## 2. 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                    上位机 (Host PC)                        │
│  python/serial_logger.py  →  串口数据采集                 │
│  python/log_to_coords.py  →  步数→柱坐标 (r,θ,z) 转换     │
└─────────────────┬────────────────────────────────────────┘
                  │ USB Serial @ 115200 baud
┌─────────────────▼────────────────────────────────────────┐
│             下位机主控 (Arduino Mega 2560)                 │
│                                                          │
│  ┌──────────┐  ┌───────────┐  ┌────────────────────┐    │
│  │ PS2 遥控 │  │ OLED 显示 │  │ CNC Shield v3       │    │
│  │ 接收模块 │  │ SSD1306   │  │ (4× A4988 Driver)   │    │
│  │ (SPI)    │  │ (I2C)     │  │                     │    │
│  └──────────┘  └───────────┘  └──────┬──┬──┬──┬─────┘    │
│                                      │  │  │  │          │
│                                   M0 M1 M2 M3             │
│                                   X  Y  Z  A              │
└─────────────────────────────────────┬──┬──┬──┬───────────┘
                                      │  │  │  │
           ┌──────────────────────────┘  │  │  └───────────┐
           ▼                             ▼  ▼               ▼
     径向伸缩电机                  旋转扫描电机  (预留)  驱动轮×3
     (独立丝杠)                  (小齿轮/内齿圈)        (1/8微步)
```

### 电机映射

| 软件索引 | CNC 轴 | 硬件功能 | 关键参数 |
|:---:|:---:|---|---|
| Motor 0 = X | X (STEP=D2, DIR=D5) | 扫描臂径向伸缩（丝杠） | 1600 步 = 全行程 |
| Motor 1 = Y | Y (STEP=D3, DIR=D6) | 360° 旋转扫描（圆盘） | 2230 步 = 360° |
| Motor 2 = Z | Z (STEP=D4, DIR=D7) | 预留 | — |
| Motor 3 = A | A (STEP=D12, DIR=D13) | 整机前进/后退（3 轮） | 1/8 微步，轮径 70mm |

> **注意：** Motor A 通过一个 A4988 同时驱动 3 个 Nema 17 驱动轮电机（接线端并联），三个轮子同步转动以维持直线行走。

---

## 3. 硬件规格

### 3.1 主控板

| 项目 | 参数 |
|---|---|
| 型号 | Arduino Mega 2560 |
| 主控芯片 | ATmega2560 |
| 主频 | 16 MHz |
| 固件框架 | Arduino (PlatformIO) |
| 串口 | USB Serial @ 115200 baud |
| 供电 | 5V 电池组（USB-B 方口） |

### 3.2 电机驱动

| 项目 | 参数 |
|---|---|
| 扩展板 | CNC Shield v3 |
| 驱动器 | 4× A4988 步进电机驱动器 |
| 逻辑电压 | 5V（12V→5V DC-DC 降压模块） |
| 动力电压 | 12V（独立电池，经继电器控制） |
| Motor A 微步 | 1/8（MS1/MS2/MS3 跳线短接） |
| 其他轴微步 | 全步（未启用微步） |

### 3.3 电源系统

采用**双电源隔离设计**：
- **5V 电池组** → Arduino Mega（USB 方口）— 主板和控制逻辑供电
- **12V 电池组** → 继电器 → A4988 ×4 — 电机动力专用
  - 12V→5V DC-DC 降压模块从动力电池取电，为 A4988 逻辑侧（VDD）供电
  - 实现动力电路与控制电路电气隔离

### 3.4 安全机制

- **继电器默认断开**：上电时引脚先写 HIGH 再设为 OUTPUT，杜绝瞬时 LOW 脉冲误触发
- **软件门控**：所有电机操作（▲/✕/R1/R2/■/↑）均检查 `relay.isOn()`，继电器断开时拒绝所有电机指令
- **OLED 警告**：拒绝指令时显示 "Access Denied" 警告，1.5 秒自动恢复

### 3.5 外设

| 组件 | 型号/协议 | 连接方式 |
|---|---|---|
| PS2 无线手柄 | 索尼 PlayStation 2 手柄 + 接收器 | SPI（软件模拟） |
| OLED 显示屏 | SSD1306 128×64 单色 | I2C（硬件，地址 0x78） |
| 继电器 | 常开式低电平触发 | D52（Active LOW） |
| MPU6050 ×2 | 6 轴 IMU（已弃用，代码保留） | I2C（地址 0x68, 0x69） |

---

## 4. 引脚映射

### CNC Shield v3 (GRBL 标准映射)

| 信号 | Arduino 引脚 |
|---|---|
| EN (全局使能) | D8 |
| X_STEP | D2 |
| X_DIR | D5 |
| Y_STEP | D3 |
| Y_DIR | D6 |
| Z_STEP | D4 |
| Z_DIR | D7 |
| A_STEP | D12 |
| A_DIR | D13 |

### PS2 手柄 (SPI)

| 信号 | Arduino 引脚 |
|---|---|
| DAT (MISO) | D22 |
| CMD (MOSI) | D24 |
| SEL (CS) | D26 |
| CLK (SCK) | D28 |

### 其他

| 信号 | Arduino 引脚 |
|---|---|
| 继电器控制 | D52 |
| OLED I2C SDA | D20 |
| OLED I2C SCL | D21 |

---

## 5. 开发环境搭建

### 5.1 前置条件

1. **PlatformIO IDE**（推荐通过 VSCode 扩展安装）
   - 安装方法：VSCode → Extensions → 搜索 "PlatformIO" → 安装
2. **Python 3.13+**（用于上位机数据采集工具）
3. **Arduino Mega 2560** 通过 USB-B 方口线连接至电脑

### 5.2 克隆项目

```bash
git clone <repo-url>
cd "FYDP Project Mega"
```

### 5.3 PlatformIO 自动配置

打开项目文件夹后，PlatformIO 会自动：
1. 识别 `platformio.ini` → 下载 `atmelavr` 平台和 `arduino` 框架
2. 扫描 `lib_deps` → 下载 TimerOne 库
3. 扫描 `lib/` → 编译本地库（U8g2, PS2X, MPU6050, I2Cdev）

### 5.4 Python 环境

```bash
cd python
python3 -m venv .venv
source .venv/bin/activate   # macOS/Linux
# .venv\Scripts\activate    # Windows
pip install -r requirements.txt
```

> `.venv/` 目录已在 `.gitignore` 中排除，不纳入版本控制。拿到项目后按上述步骤重建即可。
> 项目实际仅依赖 `pyserial`，`requirements.txt` 中明确声明了该依赖。

---

## 6. 编译与烧录

### 编译

```bash
# 在项目根目录下
pio run
```

### 烧录到 Arduino Mega

```bash
pio run --target upload
```

### 串口监视器

```bash
pio device monitor --baud 115200
```

编译产物位于 `.pio/build/megaatmega2560/firmware.hex`。

---

## 7. 使用说明

### 7.1 开机流程

1. 确认 5V 和 12V 电池组已正确连接
2. 将 Arduino Mega 通过 USB 连接至上位机（或独立 5V 供电）
3. 等待 OLED 显示 "System Ready"（约 1.5 秒）
4. PS2 手柄按 **START** 键 → 继电器闭合 → 12V 供电接入 → 电机就绪

### 7.2 PS2 手柄按键映射

| 按键 | 功能 | 触发方式 | 需继电器 |
|:---:|---|---|:---:|
| **L1** | 切换到下一个电机 | 按下瞬间 | 否 |
| **L2** | 切换到上一个电机 | 按下瞬间 | 否 |
| **R1** | 当前电机正向点动 | 按下瞬间 | ✅ |
| **R2** | 当前电机反向点动 | 按下瞬间 | ✅ |
| **▲** (三角) | 当前电机连续正向运转 | 按住不放 | ✅ |
| **✕** (叉) | 当前电机连续反向运转 | 按住不放 | ✅ |
| **■** (方块) | 启动自动化检测序列 | 按下瞬间 | ✅ |
| **↑** (方向键上) | 单步执行序列（调试） | 按下瞬间 | ✅ |
| **○** (圆圈) | 重置所有电机步数为 0 | 按下瞬间 | 否 |
| **START** | 切换继电器通断 | 按下瞬间 | 否 |
| **SELECT** | 串口打印当前步数 | 按下瞬间 | 否 |

### 7.3 R1/R2 点动步数

| 选中电机 | 每按 R1/R2 步数 | 物理意义 |
|:---:|:---:|---|
| Motor 0 (X) | ±800 | 扫描臂半行程伸缩 |
| Motor 1 (Y) | ±25 | 旋转约 4° |
| Motor 2 (Z) | ±200 | 预留 |
| Motor 3 (A) | ±100 | 前进/后退约 13.7 mm |

### 7.4 运行自动化序列

1. 按 **START** 确保继电器开启（12V 供电）
2. 按 **■** (方块) → OLED 显示 "Sequence Started" → 串口输出 `=== Automation Sequence Started ===`
3. 序列自动执行，每步完成后串口输出电机状态
4. 序列完成后串口输出 `=== Automation Sequence Finished ===`

### 7.5 典型操作流程

```
开机 → START（开继电器）→ L1/L2（选电机）→ R1/R2（点动定位）→ ■（启动序列）
```

---

## 8. 软件架构

### 8.1 核心类

| 类 | 文件 | 职责 |
|---|---|---|
| `StepperMotor` | `include/StepperMotor.h` | A4988 步进电机底层驱动（脉冲生成、三种运行模式） |
| `PS2Driver` | `include/PS2Driver.h` | PS2 手柄 16 键位掩码检测（按下/刚按下/刚松开） |
| `Relay` | `include/Relay.h` | 12V 继电器控制（Active LOW，上电安全） |
| `OledDisplay` | `include/OledDisplay.h` | SSD1306 OLED 显示（状态 + 告警覆盖层） |
| `AutomationSequence` | `include/AutomationSequence.h` | 自动化检测序列（步骤定义、单步/自动推进） |
| `DualMPU6050` | `include/DualMPU6050.h` | 双 MPU6050 IMU 姿态采集（⚠️ 已弃用） |
| `RollController` | `include/RollController.h` | 旋转角度闭环控制（⚠️ 已弃用） |

### 8.2 定时器中断架构

**这是整个系统最核心的设计决策。**

电机步进脉冲由 **Timer1 100μs 定时器中断**驱动（`motorISR()` → `updateDirectInInterrupt()`），而非在主循环中生成。原因：

- 主循环包含 PS2 通信、OLED I2C 传输、串口输出等耗时操作，实测频率仅 ~330 Hz（~3ms 周期）
- 步进电机需要微秒级脉冲时序精度 — 例如 5500μs 的脉冲间隔要求 ±100μs 误差
- 100μs 定时器中断提供比主循环高 **两个数量级** 的时间分辨率
- 每次 ISR 执行时间为纳秒级，不影响主循环

```
Timer1 ISR (每 100μs/10kHz):
  motorISR()
    ├── motorX.updateDirectInInterrupt()
    ├── motorY.updateDirectInInterrupt()
    ├── motorZ.updateDirectInInterrupt()
    └── motorA.updateDirectInInterrupt()
```

### 8.3 主循环流程

```
loop():
  1. ps2.update()          — 读取手柄，更新 16 位掩码
  2. handlePS2Input()      — 解析按键，执行对应操作
  3. oled.update()         — 刷新 OLED（仅 dirty 时）
  4. sequence.update()     — 推进自动化序列（每帧一步）
  5. 检测序列状态变化 → 串口标记
```

### 8.4 脉冲生成模式

| 模式 | 方法 | 使用场景 | 中断兼容 |
|---|---|---|---|
| 连续运转 | `run()` + ISR 中的 `updateDirectInInterrupt()` | PS2 ▲/✕ 按住不放 | ✅ |
| 阻塞式步进 | `stepBlocking()` (cli/sei 包裹) | PS2 R1/R2 点动、自动化序列 | ❌ (主动禁用) |
| 绝对位置 | `moveTo()` + ISR | 曾经使用，当前已弃用 | ✅ |

> **注意：** `stepBlocking()` 执行期间通过 `cli()` 禁用全局中断，避免与 ISR 竞争同一电机。这意味着阻塞式步进期间的 PS2 通信和 OLED 刷新会暂停（对用户体验无影响，因为步进过程极短）。

---

## 9. 数据采集管线

### 9.1 串口日志采集

```bash
cd python
source .venv/bin/activate

# 自动检测 Arduino 端口并开始记录
python serial_logger.py

# 手动指定端口
python serial_logger.py --port /dev/cu.usbmodem11201
```

- 实时显示记录行数
- Ctrl+C 停止并自动保存为 `log_data_YYYY-MM-DDTHH-MM-SS.txt`
- 每行格式：`[+<elapsed_sec>] X: <steps>, Y: <steps>, Z: <steps>, A: <steps>`

### 9.2 步数→柱坐标转换

```bash
# 修改 log_to_coords.py 中的 INPUT_FILE 为实际日志文件名
python log_to_coords.py
```

**坐标映射：**

| 坐标分量 | 来源 | 换算公式 |
|---|---|---|
| **r** (径向) | 固定值 | r = 234.0 mm（管道内半径） |
| **θ** (角度) | Motor Y | θ = steps_Y × 360° / 2230 |
| **z** (轴向) | Motor A | z = steps_A × π × 35mm / 800 |

**输出 CSV 格式：**

```csv
elapsed_s,r_mm,theta_deg,theta_rad,z_mm,x_steps,y_steps,z_steps,a_steps
0.000,234.0000,0.0000,0.000000,0.0000,0,0,0,0
1.234,234.0000,4.8421,0.084518,13.7445,0,30,0,100
```

### 9.3 可调参数

在 `log_to_coords.py` 中可根据实际硬件修改：

```python
WHEEL_RADIUS = 35.0          # [mm] 驱动轮半径
PIPE_RADIUS = 234.0          # [mm] 管道内半径
STEPS_PER_Y_REVOLUTION = 2230 # Y 电机每圈步数（实测标定值）
INPUT_FILE = "log_data_xxx.txt"
```

---

## 10. 关键技术问题与解决

### 10.1 步进电机中频共振（本项目最具挑战性的技术问题）

**现象：** 使用主循环 `update()` 驱动电机时，Motor A（驱动轮）装上橡胶轮后发生剧烈抖振——"走两步退一步"，前进效率极低。但相同的代码在空转或重载（丝杆）情况下反而正常。

**根因分析：**

1. **主循环频率瓶颈：** 主循环（含 PS2 + OLED + Serial）实测约 330 Hz（~3ms 周期）。`update()` 每次只发一个脉冲，实际脉冲间隔被锁定在 ~3ms，用户在代码中设置的 speed 参数（如 700μs）几乎不生效（"假调速"）。

2. **机械共振：** 橡胶轮增加了转子系统的转动惯量，将系统的扭转共振频率拉低至 ~330 Hz 附近，正好与主循环的等效步进频率重合。同时橡胶几乎不提供机械阻尼（与丝杆的库仑摩擦阻尼相反），振荡持续累积导致失步。

**解决方案：**

| 方案 | 内容 | 效果 |
|---|---|---|
| **软件层** | Timer1 100μs 定时器中断替代主循环 update() | 脉冲时序精度从 ±3ms → ±100μs，speed 参数真实生效 |
| **硬件层** | Motor A 启用 1/8 微步（MS1/MS2/MS3 跳线） | 消除全步冲击，从根本上避开共振条件 |

**工程经验：**
1. 不要在低速 MCU 的主循环中做精确时序控制——任何需要微秒级精度的周期信号必须交给硬件定时器中断
2. 步进电机的负载不只是"重不重"——低阻尼 + 中等惯量比大惯量 + 大阻尼更危险
3. 微步进是消除共振的有效手段——将全步拆成微步后，每一步的能量冲击大幅降低

### 10.2 双 MPU6050 闭环传感器弃用

**问题：** I2C 信号需经过约 1m 长线缆从前段传输到后段，且靠近 12V 电机 PWM 电路，信号衰减和 EMI 导致读数频繁失效。

**替代方案：** 改为纯开环步数计数——利用齿轮减速比从 Y 电机步数推算旋转角度（2230 steps = 360°，通过实际测试标定获得）。

---

## 11. 已知局限与改进方向

| 局限 | 影响 | 改进方向 |
|---|---|---|
| 旋转角度为开环控制 | 无法检测丢步，累积误差 | 差分信号编码器（RS-485）、前段独立 MCU + UART 回传 |
| 扫描臂无硬件限位 | 无法自动化"伸出→采集→收回" | 加装限位开关或接近传感器确定径向零位 |
| 自动化序列为简化版 | 当前仅做旋转扫描，无径向动作 | 待限位开关就绪后启用完整版序列（代码中已注释） |
| 三驱动轮统一控制 | 无法差速转向，依赖万向节被动过弯 | 每个驱动轮独立驱动通道实现差速控制 |
| 旋转速度编译时固定 | 无法在线调速 | 增加 PS2 按键调节速度参数的功能 |

---

## 12. 项目文件结构

```
FYDP Project Mega/
├── platformio.ini                     # PlatformIO 项目配置
├── .gitignore
├── README.md                          # 本文件
├── 02_Robot_V2_Hardware_Software_Design.md  # 详细技术文档（硬件+软件设计）
├── Final Year Project Report final_May7(1)_compressed.pdf  # 毕业设计报告全文
│
├── include/                           # C++ 头文件
│   ├── Config.h                       # 引脚定义（CNC Shield + PS2）
│   ├── StepperMotor.h                 # 步进电机驱动类
│   ├── PS2Driver.h                    # PS2 手柄驱动类
│   ├── Relay.h                        # 继电器驱动类
│   ├── OledDisplay.h                  # OLED 显示驱动类
│   ├── AutomationSequence.h           # 自动化检测序列控制器
│   ├── DualMPU6050.h                  # 双 MPU6050 IMU（⚠️ 已弃用）
│   └── RollController.h               # 旋转角度闭环控制（⚠️ 已弃用）
│
├── src/                               # C++ 实现文件
│   ├── main.cpp                       # 主程序入口
│   ├── StepperMotor.cpp
│   ├── PS2Driver.cpp
│   ├── Relay.cpp
│   ├── OledDisplay.cpp
│   ├── AutomationSequence.cpp
│   ├── DualMPU6050.cpp                # （⚠️ 已弃用）
│   └── RollController.cpp             # （⚠️ 已弃用）
│
├── python/                            # 上位机数据处理工具
│   ├── serial_logger.py               # 串口数据采集
│   ├── log_to_coords.py               # 步数→柱坐标 (r,θ,z) 转换
│   └── .venv/                         # Python 虚拟环境（不纳入版本控制）
│
├── lib/                               # 本地库（PlatformIO 自动链接）
│   ├── U8g2-2.35.30/                  # OLED 图形库
│   ├── Arduino-PS2X-master/           # PS2 手柄通信协议
│   ├── MPU6050/                       # MPU6050 IMU 驱动（⚠️ 已弃用）
│   └── I2Cdev/                        # I2C 设备驱动辅助（⚠️ 已弃用）
│
├── backup/                            # 前期代码归档（含 MPU6050 完整版 main.cpp）
├── test/                              # 前期测试代码（电机验证、MPU6050 测试等）
└── .pio/                              # PlatformIO 编译产物（不纳入版本控制）
```

---

## 13. 参考资料

1. **技术设计文档：** `02_Robot_V2_Hardware_Software_Design.md` — 硬件系统设计、软件架构、电机共振问题的详细分析
2. **毕业设计报告：** `Final Year Project Report final_May7(1)_compressed.pdf` — 项目完整论文（含文献综述、机械设计、实验结果）
3. **前期测试代码：** `test/` 目录 — 电机验证、MPU6050 测试、PS2 驱动测试等

### 依赖库文档

- [U8g2 OLED 图形库](https://github.com/olikraus/u8g2)
- [Arduino-PS2X 手柄库](https://github.com/madsci1016/Arduino-PS2X)
- [MPU6050 DMP 驱动](https://github.com/ElectronicCats/mpu6050)
- [TimerOne 定时器库](https://github.com/PaulStoffregen/TimerOne)
- [PlatformIO 文档](https://docs.platformio.org/)

---

## 许可证

This project is for academic purposes. All rights reserved.

---

*最后更新：2026.05.30*
