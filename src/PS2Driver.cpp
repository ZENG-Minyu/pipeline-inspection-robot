/*
  PS2 无线手柄驱动模块 — 实现
  ============================
  基于 PS2X_lib 库，通过 16 位掩码实现高效的多按键状态追踪和边沿检测。

  位掩码机制：
  - 每帧调用 update() 读取 ps2x 状态，将 16 个按键的按下/松开状态编码为 uint16_t
  - 通过上一帧和当前帧掩码的差分，实现下降沿和上升沿检测
  - 位运算复杂度 O(1)，适合在 Arduino Mega 的主循环中高频调用

  作者：ZENG Minyu
*/
#include "PS2Driver.h"

// ============================================================
// 构造函数
// ============================================================

PS2Driver::PS2Driver(int clk, int cmd, int att, int dat) {
    _clk = clk;
    _cmd = cmd;
    _att = att;
    _dat = dat;
    _pressedMask = 0;
    _lastPressedMask = 0;
}

// ============================================================
// 初始化
// ============================================================

bool PS2Driver::init() {
    // config_gamepad(时钟, 命令, 片选, 数据, 压力感应, 震动)
    // 返回 0 表示配对成功
    int error = ps2x.config_gamepad(_clk, _cmd, _att, _dat, true, true);

    if (error == 0) {
        return true;
    } else {
        return false;
    }
}

// ============================================================
// 每帧更新
// ============================================================

void PS2Driver::update() {
    _lastPressedMask = _pressedMask;  // 保存上一帧状态用于边沿检测
    _pressedMask = 0;

    ps2x.read_gamepad(false, 0);  // false = 非阻塞读取, 0 = 普通手柄模式

    // 遍历 16 个按键，构建当前帧位掩码
    for (int i = 0; i < MAX_BUTTONS; i++) {
        if (ps2x.Button(_buttons[i])) {
            _pressedMask |= (1 << i);
        }
    }
}

// ============================================================
// 按键状态查询（基于位掩码）
// ============================================================

int PS2Driver::findIndex(uint16_t button) {
    for (int i = 0; i < MAX_BUTTONS; i++) {
        if (_buttons[i] == button) return i;
    }
    return -1;
}

bool PS2Driver::pressed(uint16_t button) {
    int idx = findIndex(button);
    if (idx < 0) return false;
    return (_pressedMask >> idx) & 1;
}

bool PS2Driver::justPressed(uint16_t button) {
    int idx = findIndex(button);
    if (idx < 0) return false;
    uint16_t mask = (1 << idx);
    // 上一帧未按 AND 当前帧按下 → 下降沿
    return !(_lastPressedMask & mask) && (_pressedMask & mask);
}

bool PS2Driver::justReleased(uint16_t button) {
    int idx = findIndex(button);
    if (idx < 0) return false;
    uint16_t mask = (1 << idx);
    // 上一帧按下 AND 当前帧未按 → 上升沿
    return (_lastPressedMask & mask) && !(_pressedMask & mask);
}
