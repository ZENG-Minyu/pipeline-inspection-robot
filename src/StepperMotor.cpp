/*
  步进电机驱动模块 — 实现
  ========================
  所有步进电机操作的底层实现，包括脉冲生成、位置追踪和运行模式控制。

  脉冲生成时序：
  STEP 引脚：HIGH（持续 pulseWidthUs）→ LOW（持续 intervalUs - pulseWidthUs）
  重复生成，直至到达目标位置或手动停止。

  中断安全设计：
  - stepBlocking() 使用 cli()/sei() 禁用全局中断，防止与 Timer1 ISR 中的
    updateDirectInInterrupt() 同时操作同一电机引脚造成冲突
  - resetPosition() 同样使用 cli()/sei() 保护 _currentPosition 的原子性

  作者：ZENG Minyu
*/
#include "StepperMotor.h"
#include <avr/interrupt.h>

// ============================================================
// 构造函数 & 初始化
// ============================================================

StepperMotor::StepperMotor(int stepPin, int dirPin) {
    _stepPin = stepPin;
    _dirPin = dirPin;

    _currentPosition = 0;
    _targetPosition = 0;

    _intervalUs = 800;       // 默认脉冲间隔 800μs（约 1.25kHz）
    _lastStepTime = 0;
    _pulseWidthUs = 200;     // 默认脉冲宽度 200μs，确保 A4988 可靠检测上升沿

    _running = false;
    _continuousMode = false;
}

void StepperMotor::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
}

// ============================================================
// 绝对位置模式
// ============================================================

void StepperMotor::moveTo(long targetPosition, unsigned long intervalUs) {
    _targetPosition = targetPosition;
    _intervalUs = intervalUs;
    _continuousMode = false;

    // 根据目标位置与当前位置的差值决定转向
    if (_targetPosition > _currentPosition) {
        _dir = CW;
    } else if (_targetPosition < _currentPosition) {
        _dir = CCW;
    } else {
        _running = false;  // 已到达目标位置，无需运动
        return;
    }

    digitalWrite(_dirPin, _dir);
    _lastStepTime = micros();
    _running = true;
}

// ============================================================
// 连续运转模式
// ============================================================

void StepperMotor::run(Direction dir, unsigned long intervalUs) {
    // 仅在参数变化或首次启动时重置状态，避免不必要的方向引脚写入和计时器重置
    if (!_running || _dir != dir || _intervalUs != intervalUs) {
        _dir = dir;
        _intervalUs = intervalUs;
        _continuousMode = true;

        digitalWrite(_dirPin, _dir);
        _lastStepTime = micros();  // 重置计时基准，避免累积误差
        _running = true;
    }
}

void StepperMotor::stop() {
    _running = false;
}

// ============================================================
// 主循环更新（非中断模式，已弃用）
// ============================================================

void StepperMotor::update() {
    if (!_running) return;

    unsigned long now = micros();

    // 检查是否到达下一次脉冲的时间
    if (now - _lastStepTime >= _intervalUs) {
        _lastStepTime = now;

        stepOnce();

        if (_dir == CW) {
            _currentPosition++;
        } else {
            _currentPosition--;
        }

        // 位置模式：到达目标后自动停止
        if (!_continuousMode) {
            if ((_dir == CW && _currentPosition >= _targetPosition) ||
                (_dir == CCW && _currentPosition <= _targetPosition)) {
                _running = false;
            }
        }
    }
}

// ============================================================
// 脉冲生成底层
// ============================================================

void StepperMotor::stepOnce() {
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(_pulseWidthUs);  // 维持高电平以确保 A4988 检测到上升沿
    digitalWrite(_stepPin, LOW);
}

// ============================================================
// 状态查询
// ============================================================

bool StepperMotor::isRunning() const {
    return _running;
}

bool StepperMotor::isContinuousMode() const {
    return _continuousMode;
}

long StepperMotor::getCurrentPosition() const {
    return _currentPosition;
}

// ============================================================
// 位置重置（中断安全）
// ============================================================

void StepperMotor::resetPosition() {
    cli();                      // 禁用全局中断
    _currentPosition = 0;       // 临界区：防止 ISR 同时读写此变量
    sei();                      // 恢复全局中断
}

// ============================================================
// 阻塞式步进
// ============================================================

void StepperMotor::stepBlocking(long steps, unsigned long intervalUs, bool (*shouldContinue)()) {
    if (steps == 0) return;

    cli();  // 禁用中断，避免与 updateDirectInInterrupt() 竞争同一电机

    // 从总间隔中扣除脉冲宽度得到步间等待时间
    // 例如：intervalUs=6000, _pulseWidthUs=200 → waitTimeUs=5800
    unsigned long waitTimeUs = intervalUs - _pulseWidthUs;
    if (waitTimeUs > intervalUs) {  // 防止 unsigned 下溢
        waitTimeUs = 1;
    }

    Direction dir = (steps > 0) ? CW : CCW;
    long absSteps = abs(steps);

    digitalWrite(_dirPin, dir);

    for (long i = 0; i < absSteps; i++) {
        // 检查外部中断条件（当前未使用，预留接口）
        if (shouldContinue && !shouldContinue()) {
            sei();
            return;
        }

        stepOnce();
        if (dir == CW) {
            _currentPosition++;
        } else {
            _currentPosition--;
        }
        delayMicroseconds(waitTimeUs);
    }

    sei();  // 恢复全局中断
}

// ============================================================
// 定时器中断更新（当前主要使用的脉冲生成方式）
// ============================================================

void StepperMotor::updateDirectInInterrupt() {
    if (!_running) return;

    unsigned long now = micros();

    // 在 ISR 中检查时间是否到达下一次脉冲
    if (now - _lastStepTime >= _intervalUs) {
        _lastStepTime = now;

        // 直接在 ISR 中生成完整脉冲（设计为纳秒级执行，不阻塞其他中断）
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(_pulseWidthUs);
        digitalWrite(_stepPin, LOW);

        if (_dir == CW) {
            _currentPosition++;
        } else {
            _currentPosition--;
        }

        // 位置模式：到达目标后自动停止
        if (!_continuousMode) {
            if ((_dir == CW && _currentPosition >= _targetPosition) ||
                (_dir == CCW && _currentPosition <= _targetPosition)) {
                _running = false;
            }
        }
    }
}

// ============================================================
// 参数配置
// ============================================================

void StepperMotor::setPulseWidth(unsigned long pulseWidthUs) {
    _pulseWidthUs = pulseWidthUs;
}
