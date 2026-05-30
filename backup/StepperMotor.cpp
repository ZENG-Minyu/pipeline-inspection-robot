#include "StepperMotor.h"
#include <avr/interrupt.h>

// 静态成员初始化
StepperMotor* StepperMotor::_motors[StepperMotor::MAX_MOTORS] = {nullptr};
int StepperMotor::_motorCount = 0;

StepperMotor::StepperMotor(int stepPin, int dirPin) {
    _stepPin = stepPin;
    _dirPin = dirPin;

    _currentPosition = 0;
    _targetPosition = 0;

    _intervalUs = 800;
    _lastStepTime = 0;
    _pulseStartTime = 0;
    _pulseWidthUs = 10;  // 默认脉冲宽度10微秒

    _running = false;
    _continuousMode = false;
    _pulseActive = false;
}

void StepperMotor::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
}

void StepperMotor::moveTo(long targetPosition, unsigned long intervalUs) {
    _targetPosition = targetPosition;
    _intervalUs = intervalUs;

    _continuousMode = false;

    if (_targetPosition > _currentPosition) {
        _dir = CW;
    } else if (_targetPosition < _currentPosition) {
        _dir = CCW;
    } else {
        _running = false;
        return;
    }

    digitalWrite(_dirPin, _dir);
    _lastStepTime = micros();
    _running = true;
}

void StepperMotor::run(Direction dir, unsigned long intervalUs) {
    if (!_running || _dir != dir || _intervalUs != intervalUs) {

        _dir = dir;
        _intervalUs = intervalUs;
        _continuousMode = true;

        digitalWrite(_dirPin, _dir);

        _lastStepTime = micros();   // 只在变化时重置
        _running = true;
    }
}

void StepperMotor::stop() {
    _running = false;
}


void StepperMotor::update() {
    if (!_running) return;

    unsigned long now = micros();

    if (now - _lastStepTime >= _intervalUs) {
        _lastStepTime = now;

        stepOnce();

        if (_dir == CW) {
            _currentPosition++;
        } else {
            _currentPosition--;
        }

        // 只有位置模式才判断停止
        if (!_continuousMode) {
            if ((_dir == CW && _currentPosition >= _targetPosition) ||
                (_dir == CCW && _currentPosition <= _targetPosition)) {
                _running = false;
            }
        }
    }
}

void StepperMotor::stepOnce() {
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(_pulseWidthUs);  // 使用可配置的脉冲宽度
    digitalWrite(_stepPin, LOW);
}

bool StepperMotor::isRunning() const {
    return _running;
}

bool StepperMotor::isContinuousMode() const {
    return _continuousMode;
}

long StepperMotor::getCurrentPosition() const {
    return _currentPosition;
}

void StepperMotor::resetPosition() {
    // 禁用中断以避免与中断更新冲突
    cli();
    _currentPosition = 0;
    sei();
}

void StepperMotor::stepBlocking(long steps, unsigned long intervalUs) {
    if (steps == 0) return;

    // 禁用中断以避免与中断更新冲突
    cli();

    // intervalUs参数现在表示总间隔时间（包含脉冲宽度）
    // 扣除脉冲宽度得到步进之间的等待时间
    unsigned long waitTimeUs = intervalUs - _pulseWidthUs;
    if (waitTimeUs > intervalUs) {  // 防止下溢
        waitTimeUs = 1;
    }

    Direction dir = (steps > 0) ? CW : CCW;
    long absSteps = abs(steps);

    digitalWrite(_dirPin, dir);

    for (long i = 0; i < absSteps; i++) {
        stepOnce();
        if (dir == CW) {
            _currentPosition++;
        } else {
            _currentPosition--;
        }
        delayMicroseconds(waitTimeUs);
    }

    // 恢复中断
    sei();
}

void StepperMotor::registerMotor(StepperMotor* motor) {
    if (_motorCount < MAX_MOTORS) {
        _motors[_motorCount++] = motor;
    }
}

void StepperMotor::updateAllInInterrupt() {
    for (int i = 0; i < _motorCount; i++) {
        if (_motors[i]) {
            _motors[i]->updateInInterrupt();
        }
    }
}

void StepperMotor::updateInInterrupt() {
    // 如果脉冲活跃，检查脉冲宽度是否达到配置值
    if (_pulseActive) {
        unsigned long now = micros();
        if (now - _pulseStartTime >= _pulseWidthUs) {
            digitalWrite(_stepPin, LOW);
            _pulseActive = false;
        } else {
            // 脉冲尚未结束，等待下一个中断
            return;
        }
    }

    if (!_running) return;

    unsigned long now = micros();

    // 检查是否达到步进间隔
    if (now - _lastStepTime >= _intervalUs) {
        _lastStepTime = now;

        // 生成步进脉冲
        stepPulse();

        // 更新位置
        if (_dir == CW) {
            _currentPosition++;
        } else {
            _currentPosition--;
        }

        // 位置模式：检查是否达到目标
        if (!_continuousMode) {
            if ((_dir == CW && _currentPosition >= _targetPosition) ||
                (_dir == CCW && _currentPosition <= _targetPosition)) {
                _running = false;
            }
        }
    }
}

void StepperMotor::stepPulse() {
    digitalWrite(_stepPin, HIGH);
    _pulseStartTime = micros();
    _pulseActive = true;
}

void StepperMotor::setPulseWidth(unsigned long pulseWidthUs) {
    _pulseWidthUs = pulseWidthUs;
}