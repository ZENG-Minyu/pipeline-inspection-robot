/*
  步进电机驱动
  更新日期：2026.3.25
  作者：ZENG,Minyu
*/
#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>

enum Direction {
    CW = HIGH,
    CCW = LOW
};

class StepperMotor {
public:
    StepperMotor(int stepPin, int dirPin);

    void begin();

    // 设置目标位置（绝对位置）
    void moveTo(long targetPosition, unsigned long intervalUs);

    // 连续模式
    void run(Direction dir, unsigned long intervalUs);
    void stop();

    // 每次 loop 调用（核心）- 保留用于非中断模式
    void update();

    // 中断安全的更新函数
    void updateInInterrupt();

    // 是否还在运动
    bool isRunning() const;

    //是否在连续模式
    bool isContinuousMode() const;

    // 获取当前位置
    long getCurrentPosition() const;

    // 重置位置
    void resetPosition();

    // 阻塞式走指定步数
    void stepBlocking(long steps, unsigned long intervalUs);

    // 静态方法用于中断服务程序
    static void registerMotor(StepperMotor* motor);
    static void updateAllInInterrupt();

    // 设置脉冲宽度（微秒）
    void setPulseWidth(unsigned long pulseWidthUs);

private:
    void stepOnce();
    void stepPulse();  // 中断安全的步进脉冲生成

    int _stepPin;
    int _dirPin;

    long _currentPosition;
    long _targetPosition;

    unsigned long _intervalUs;
    unsigned long _lastStepTime;
    unsigned long _pulseStartTime;  // 脉冲开始时间
    unsigned long _pulseWidthUs;    // 脉冲宽度（微秒）

    bool _running;
    bool _continuousMode;
    bool _pulseActive;  // 脉冲是否活跃
    Direction _dir;

    // 静态成员用于中断
    static const int MAX_MOTORS = 8;
    static StepperMotor* _motors[MAX_MOTORS];
    static int _motorCount;
};

#endif