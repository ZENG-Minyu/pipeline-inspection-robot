/*
  OLED 显示屏驱动模块 — 实现
  ============================
  基于 U8g2 库的页缓冲模式驱动 SSD1306 OLED。

  设计要点：
  - 使用 U8g2 第一页缓冲区模式（_1_），内存占用最低（128 bytes/page）
  - firstPage()/nextPage() 循环是 U8g2 的标准渲染范式，每次渲染需完整遍历
  - 告警覆盖层通过时间戳自动过期，过期后自动恢复为电机状态显示

  作者：ZENG Minyu
*/
#include "OledDisplay.h"
#include <Wire.h>

// ============================================================
// 构造函数
// ============================================================

OledDisplay::OledDisplay()
    : _u8g2(U8G2_R0, U8X8_PIN_NONE),
      _motorIndex(0),
      _position(0),
      _dirty(true),
      _alertActive(false),
      _alertUntilMs(0),
      _alertLine1(nullptr),
      _alertLine2(nullptr) {
}

// ============================================================
// 初始化
// ============================================================

void OledDisplay::begin() {
    Wire.begin();
    Wire.setClock(100000);       // I2C 标准模式 100kHz（OLED 不需要高速）
    _u8g2.setI2CAddress(0x78);  // SSD1306 默认 I2C 地址（SA0 = 0）
    _u8g2.begin();
    _u8g2.enableUTF8Print();     // 启用 UTF-8 字符支持
    _u8g2.setFont(u8g2_font_10x20_te);  // 10×20 像素字体，适合 128×64 屏幕
    _u8g2.setPowerSave(0);       // 禁用省电模式（正常显示）
    clear();
    _dirty = true;
}

void OledDisplay::clear() {
    _u8g2.firstPage();
    do {
        // 空循环：u8g2 在 firstPage/nextPage 之间默认清空缓冲区
    } while (_u8g2.nextPage());
}

// ============================================================
// 启动画面 & 状态文本
// ============================================================

void OledDisplay::showBootScreen() {
    showStatusText("System Ready");
}

void OledDisplay::showStatusText(const char* line1, const char* line2) {
    _u8g2.firstPage();
    do {
        drawCenteredTextInternal(20, line1);
        if (line2 != nullptr) {
            drawCenteredTextInternal(52, line2);
        }
    } while (_u8g2.nextPage());
}

void OledDisplay::showSystemReady() {
    showBootScreen();
}

// ============================================================
// 电机状态显示
// ============================================================

void OledDisplay::showMotorStatus(int motorIndex, long position) {
    renderMotorStatus(motorIndex, position);
}

void OledDisplay::showMotorStatus(const char* motorName, const char* status) {
    _u8g2.firstPage();
    do {
        _u8g2.drawStr(0, 14, "Motor:");
        _u8g2.drawStr(48, 14, motorName);
        _u8g2.drawStr(0, 34, "Status:");
        _u8g2.drawStr(56, 34, status);
    } while (_u8g2.nextPage());
}

void OledDisplay::renderMotorStatus(int motorIndex, long position) {
    char motorLine[24];
    char positionLine[24];

    snprintf(motorLine, sizeof(motorLine), "Motor %d", motorIndex);
    snprintf(positionLine, sizeof(positionLine), "Pos %ld", position);

    _u8g2.firstPage();
    do {
        drawCenteredTextInternal(24, motorLine);
        drawCenteredTextInternal(56, positionLine);
    } while (_u8g2.nextPage());
}

// ============================================================
// 继电器状态显示
// ============================================================

void OledDisplay::showRelayStatus(bool on) {
    _u8g2.firstPage();
    do {
        _u8g2.drawStr(0, 14, "Relay");
        _u8g2.drawStr(0, 34, "State:");
        _u8g2.drawStr(56, 34, on ? "ON" : "OFF");
    } while (_u8g2.nextPage());
}

// ============================================================
// 告警覆盖层
// ============================================================

void OledDisplay::showAccessDenied(const char* line1, const char* line2, unsigned long durationMs) {
    _alertActive = true;
    _alertUntilMs = millis() + durationMs;
    _alertLine1 = line1;
    _alertLine2 = line2;
    _dirty = true;  // 立即触发刷新
}

void OledDisplay::renderAccessDenied(const char* line1, const char* line2) {
    _u8g2.firstPage();
    do {
        drawCenteredTextInternal(22, line1);
        if (line2 != nullptr) {
            drawCenteredTextInternal(54, line2);
        }
    } while (_u8g2.nextPage());
}

// ============================================================
// 状态管理与刷新
// ============================================================

void OledDisplay::setMotorState(int motorIndex, long position) {
    _motorIndex = motorIndex;
    _position = position;
    _dirty = true;
}

void OledDisplay::requestRefresh() {
    _dirty = true;
}

void OledDisplay::update() {
    // 优先处理告警覆盖层
    if (_alertActive) {
        // 使用带符号减法检测时间是否过期（正确处理 millis() 溢出回绕）
        if ((long)(millis() - _alertUntilMs) < 0) {
            renderAccessDenied(_alertLine1, _alertLine2);
            return;  // 告警激活期间不渲染正常状态
        }
        // 告警已过期：自动恢复
        _alertActive = false;
        _dirty = true;
    }

    // 仅在内容变化时刷新（减少 I2C 通信开销）
    if (!_dirty) return;
    renderMotorStatus(_motorIndex, _position);
    _dirty = false;
}

// ============================================================
// 辅助：居中文本绘制
// ============================================================

void OledDisplay::drawCenteredTextInternal(int y, const char* text) {
    if (text == nullptr) return;
    int16_t w = _u8g2.getStrWidth(text);
    int16_t x = (128 - w) / 2;  // OLED 宽度 128px
    if (x < 0) x = 0;           // 文本超宽时左对齐
    _u8g2.drawStr(x, y, text);
}
