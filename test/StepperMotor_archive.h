/*
  阻塞式步进电机驱动
  更新日期：2026.3.6
  作者：ZENG,Minyu
*/
#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>

enum Direction {
    CW = HIGH,   // 顺时针
    CCW = LOW    // 逆时针
};

class StepperMotor {
public:
    // 构造函数：初始化引脚
    StepperMotor(int stepPin, int dirPin);

    // 初始化方法：设置 pinMode（通常在 setup 中调用）
    void begin();

    // 运动控制：步数、方向、每步间隔（微秒）
    //建议intervalUs=[400,800],过小会丢步，过大会抖动
    void move(int steps, Direction dir, unsigned long intervalUs);

    // 获取当前绝对位置
    long getCurrentPosition() const;

    // 重置位置计数
    void resetPosition();

private:
    int _stepPin;
    int _dirPin;
    long _currentPosition; // 记录电机的绝对步数位置
};

#endif