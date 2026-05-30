/*
  OLED 显示屏驱动模块
  ===================
  驱动 SSD1306 128×64 像素单色 OLED 显示屏，通过 I2C 总线通信。

  硬件规格：
  - 型号：SSD1306 128×64 单色 OLED
  - 通信：I2C（硬件 I2C，SDA = D20, SCL = D21），地址 0x78
  - 库：U8g2（页缓冲模式，内存占用最低）

  显示内容层级：
  - 正常状态层：显示当前选中的电机编号和实时步数
  - 告警覆盖层：Access Denied 警告界面（继电器断开时触发，自动 1.5 秒后恢复）

  刷新机制：
  - _dirty 标志位 + requestRefresh()：避免不必要的屏幕刷新
  - 告警状态通过 _alertActive 和时间戳实现自动过期恢复

  作者：ZENG Minyu
*/
#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>
#include <U8g2lib.h>

class OledDisplay {
public:
    OledDisplay();

    // 初始化 I2C 总线和 OLED 显示参数（字体、I2C 地址等）
    void begin();

    // 清空屏幕（写空白页）
    void clear();

    // 显示启动画面（"System Ready"）
    void showBootScreen();

    // 显示通用状态文本（1-2 行，居中显示）
    void showStatusText(const char* line1, const char* line2 = nullptr);

    // 显示指定电机的状态（编号 + 步数）
    void showMotorStatus(int motorIndex, long position);
    void showMotorStatus(const char* motorName, const char* status);

    // 显示系统就绪（等同 showBootScreen）
    void showSystemReady();

    // 显示继电器状态（ON/OFF）
    void showRelayStatus(bool on);

    // 显示"拒绝访问"警告界面（继电器断开时触发）
    // durationMs：警告显示持续时间（毫秒），默认 1500ms
    void showAccessDenied(const char* line1, const char* line2, unsigned long durationMs = 1500);

    // 设置当前电机状态（编号 + 位置），标记需要刷新
    void setMotorState(int motorIndex, long position);

    // 标记显示内容已变化，在下次 update() 时刷新
    void requestRefresh();

    // 每帧更新：检查告警是否过期 → 按需渲染
    void update();

private:
    // U8g2 实例：SSD1306 128×64, 第一页缓冲区模式（内存占用最小）
    // U8G2_R0 = 不旋转, U8X8_PIN_NONE = 硬件 I2C 无需指定引脚
    U8G2_SSD1306_128X64_NONAME_1_HW_I2C _u8g2;

    int _motorIndex;           // 当前选中的电机编号 (0-3)
    long _position;            // 当前电机的步数
    bool _dirty;               // 是否需要刷新屏幕
    bool _alertActive;         // 告警覆盖层是否活跃
    unsigned long _alertUntilMs;  // 告警过期时间戳（millis()）
    const char* _alertLine1;   // 告警文本第一行
    const char* _alertLine2;   // 告警文本第二行

    // 在指定 y 坐标居中绘制文本
    void drawCenteredTextInternal(int y, const char* text);

    // 渲染正常状态：电机编号 + 步数
    void renderMotorStatus(int motorIndex, long position);

    // 渲染告警覆盖层：警告信息
    void renderAccessDenied(const char* line1, const char* line2);
};

#endif
