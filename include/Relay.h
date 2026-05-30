/*
  常开式低电平触发继电器驱动模块
  ==============================
  控制 12V 电机动力电源的通断，实现硬件级安全保护。

  继电器规格：
  - 类型：常开式（Normally Open），低电平触发（Active LOW）
  - 控制引脚：Arduino Mega D52
  - 逻辑：LOW = 通路（12V 送至 A4988），HIGH = 断路（12V 断开）

  安全机制：
  - 上电初始化时先写 HIGH 再设 OUTPUT 模式，杜绝引脚默认 LOW 产生的瞬时误触发
  - 所有电机操作（▲/✕/R1/R2/■/↑）均受 isOn() 门控，继电器断开时电机指令被拒绝

  作者：ZENG Minyu
*/
#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>

class Relay {
public:
    // 构造函数：传入控制引脚编号
    Relay(int pin);

    // 初始化引脚：先写 HIGH（断路），再设为 OUTPUT，防止上电误触发
    void begin();

    // 开启继电器：写 LOW（通路），12V 电源接通
    void turnOn();

    // 关闭继电器：写 HIGH（断路），12V 电源断开
    void turnOff();

    // 切换继电器状态（ON → OFF 或 OFF → ON）
    void toggle();

    // 查询当前状态：true = 通路（12V 供电中），false = 断路
    bool isOn() const;

private:
    int _pin;      // 控制引脚编号
    bool _isOn;    // 内部状态标志
};

#endif
