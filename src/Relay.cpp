/*
  常开式低电平触发继电器驱动模块 — 实现
  ======================================
  控制 12V 动力电源的通断。详见头文件说明。

  作者：ZENG Minyu
*/
#include "Relay.h"

Relay::Relay(int pin) : _pin(pin), _isOn(false) {
}

void Relay::begin() {
    // 关键安全设计：先写 HIGH 确保继电器断开，再设为 OUTPUT
    // 如果顺序反过来（先设 OUTPUT 再写 HIGH），引脚默认 LOW 会在瞬间触发通路
    digitalWrite(_pin, HIGH);
    pinMode(_pin, OUTPUT);
}

void Relay::turnOn() {
    digitalWrite(_pin, LOW);  // 低电平触发：LOW = 通路
    _isOn = true;
}

void Relay::turnOff() {
    digitalWrite(_pin, HIGH);  // 高电平断开：HIGH = 断路
    _isOn = false;
}

void Relay::toggle() {
    if (_isOn) {
        turnOff();
    } else {
        turnOn();
    }
}

bool Relay::isOn() const {
    return _isOn;
}
