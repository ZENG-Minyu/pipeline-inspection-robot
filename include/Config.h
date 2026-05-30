/*
  硬件引脚配置文件
  ================
  定义 CNC Shield v3 扩展板和 PS2 无线手柄在 Arduino Mega 2560 上的引脚映射。
  所有引脚定义集中在本文档中，方便硬件接线变更时统一修改。

  硬件平台：Arduino Mega 2560 (ATmega2560, 16MHz)
  扩展板：CNC Shield v3（搭载 4 路 A4988 步进电机驱动器）
  作者：ZENG Minyu
*/
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// CNC Shield v3 引脚定义
// ============================================================
// CNC Shield v3 通过 Arduino Mega 的 GRBL 标准引脚映射进行连接。
// 四个轴（X/Y/Z/A）各使用两个引脚：STEP（步进脉冲）和 DIR（方向）。
// 所有轴的使能信号共用同一个 EN 引脚，低电平使能。

#define PIN_EN        8   // 全局使能引脚（LOW = 全部四轴使能）

// X 轴 — Motor 0：探头径向伸缩（扫描臂升降丝杠）
#define PIN_STEP_X    2   // X 轴步进脉冲
#define PIN_DIR_X     5   // X 轴方向控制

// Y 轴 — Motor 1：360° 旋转扫描（旋转圆盘，小齿轮驱动内齿圈）
#define PIN_STEP_Y    3   // Y 轴步进脉冲
#define PIN_DIR_Y     6   // Y 轴方向控制

// Z 轴 — Motor 2：预留（当前未使用）
#define PIN_STEP_Z    4   // Z 轴步进脉冲
#define PIN_DIR_Z     7   // Z 轴方向控制

// A 轴 — Motor 3：整机前进/后退（3 轮同步驱动，接线端并联）
// A4988 跳线配置为 1/8 微步模式（MS1/MS2/MS3 均短接）
#define PIN_STEP_A    12  // A 轴步进脉冲
#define PIN_DIR_A     13  // A 轴方向控制

// ============================================================
// PS2 无线手柄引脚定义（SPI 通信）
// ============================================================
// PS2 手柄接收器通过 SPI 协议与 Arduino 通信。
// 注意：Arduino Mega 的硬件 SPI 引脚为 D50(MISO)/D51(MOSI)/D52(SCK)/D53(SS)，
// 但 PS2X 库使用软件模拟 SPI，因此可以使用任意数字引脚。

#define PS2_DAT       22  // 数据线（MISO，手柄 → Arduino）
#define PS2_CMD       24  // 命令线（MOSI，Arduino → 手柄）
#define PS2_SEL       26  // 片选线（CS  / SS，选中手柄通信）
#define PS2_CLK       28  // 时钟线（SCK，同步通信时序）

#endif
