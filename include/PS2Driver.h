/*
  PS2 无线手柄驱动模块
  ====================
  封装 PS2X_lib 库，提供按键状态的位掩码检测系统。

  通信接口：SPI（软件模拟，非硬件 SPI）
  支持的按键：16 个（SELECT/L3/R3/START/方向键×4/L2/R2/L1/R1/功能键×4）

  三种按键检测模式（通过位掩码差分实现）：
  - pressed()：当前帧该键被按住（持续按住为 true）
  - justPressed()：下降沿检测（按下瞬间为 true，用于触发一次性动作）
  - justReleased()：上升沿检测（松开瞬间为 true）

  设计意义：区分"按住"和"刚按下"避免了按键重复触发。
  例如 ▲/✕ 持续运转用 pressed()，R1/R2 点动用 justPressed()。

  作者：ZENG Minyu
*/
#ifndef PS2_DRIVER_H
#define PS2_DRIVER_H

#include <PS2X_lib.h>
#include <Arduino.h>

#define MAX_BUTTONS 16  // PS2 手柄可检测的按键总数

class PS2Driver {
private:
    PS2X ps2x;           // 底层 PS2X 库实例
    int _clk, _cmd, _att, _dat;  // SPI 引脚

    // 16 个按键的 PS2X 库定义码，按索引顺序排列（索引 0 对应位掩码 bit 0）：
    // 0=SELECT,  1=L3,    2=R3,    3=START,
    // 4=↑(UP),   5=→(RIGHT), 6=↓(DOWN), 7=←(LEFT),
    // 8=L2,      9=R2,    10=L1,   11=R1,
    // 12=▲(TRIANGLE), 13=○(CIRCLE), 14=✕(CROSS), 15=■(SQUARE)
    uint16_t _buttons[MAX_BUTTONS] = {
        PSB_SELECT, PSB_L3, PSB_R3, PSB_START,
        PSB_PAD_UP, PSB_PAD_RIGHT, PSB_PAD_DOWN, PSB_PAD_LEFT,
        PSB_L2, PSB_R2, PSB_L1, PSB_R1,
        PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE
    };

    uint16_t _pressedMask;      // 当前帧按键位掩码（bit i = 1 表示按键 i 被按住）
    uint16_t _lastPressedMask;  // 上一帧按键位掩码（用于边沿检测）

public:
    // 构造函数：传入 SPI 通信的 4 个引脚
    // clk — 时钟线，cmd — 命令线（MOSI），att — 片选线（CS），dat — 数据线（MISO）
    PS2Driver(int clk, int cmd, int att, int dat);

    // 初始化手柄：调用 PS2X 库的 config_gamepad 进行配对
    // 启用压力感应 (true) 和震动反馈 (true)
    // 返回 true 表示配对成功
    bool init();

    // 每帧更新：读取手柄状态并更新位掩码
    // 应在每次 loop() 开始时调用
    void update();

    // 按键持续按住检测（当前帧按下即为 true）
    bool pressed(uint16_t button);

    // 按键下降沿检测（上一帧未按 → 当前帧按下）
    bool justPressed(uint16_t button);

    // 按键上升沿检测（上一帧按下 → 当前帧未按）
    bool justReleased(uint16_t button);

private:
    // 根据 PS2X 库的按键码查找在 _buttons 数组中的索引
    int findIndex(uint16_t button);
};

#endif
