#include "StepperMotor.h"

StepperMotor::StepperMotor(int stepPin, int dirPin) {
    _stepPin = stepPin;
    _dirPin = dirPin;
    _currentPosition = 0;
}

void StepperMotor::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
}

void StepperMotor::move(int steps, Direction dir, unsigned long intervalUs) {
    // 设置方向引脚
    digitalWrite(_dirPin, dir);

    for (int i = 0; i < steps; i++) {
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(intervalUs);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(intervalUs);

        // 更新坐标：如果是顺时针则增加，逆时针则减少
        if (dir == CW) {
            _currentPosition++;
        } else {
            _currentPosition--;
        }
    }
}

long StepperMotor::getCurrentPosition() const {
    return _currentPosition;
}

void StepperMotor::resetPosition() {
    _currentPosition = 0;
}