#include "PS2Driver.h"

PS2Driver::PS2Driver(int clk, int cmd, int att, int dat) {
    _clk = clk;
    _cmd = cmd;
    _att = att;
    _dat = dat;

    for (int i = 0; i < MAX_BUTTONS; i++) {
        _currentState[i] = false;
        _lastState[i] = false;
    }
}

bool PS2Driver::init() {
    int error = ps2x.config_gamepad(_clk, _cmd, _att, _dat, true, true);

    if (error == 0) {
        Serial.println("Gamepad found!");
        return true;
    } else {
        Serial.print("Error configuring gamepad: ");
        Serial.println(error);
        return false;
    }
}

void PS2Driver::update() {

    // 保存上一帧
    for (int i = 0; i < MAX_BUTTONS; i++) {
        _lastState[i] = _currentState[i];
    }

    ps2x.read_gamepad(false, 0);

    // 更新当前状态
    for (int i = 0; i < MAX_BUTTONS; i++) {
        _currentState[i] = ps2x.Button(_buttons[i]);
    }
}

int PS2Driver::findIndex(uint16_t button) {
    for (int i = 0; i < MAX_BUTTONS; i++) {
        if (_buttons[i] == button) return i;
    }
    return -1;
}

bool PS2Driver::pressed(uint16_t button) {
    int idx = findIndex(button);
    if (idx < 0) return false;
    return _currentState[idx];
}

bool PS2Driver::justPressed(uint16_t button) {
    int idx = findIndex(button);
    if (idx < 0) return false;
    return (!_lastState[idx] && _currentState[idx]);
}

bool PS2Driver::justReleased(uint16_t button) {
    int idx = findIndex(button);
    if (idx < 0) return false;
    return (_lastState[idx] && !_currentState[idx]);
}