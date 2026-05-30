#ifndef PS2_DRIVER_H
#define PS2_DRIVER_H

#include <PS2X_lib.h>
#include <Arduino.h>

#define MAX_BUTTONS 16

class PS2Driver {
private:
    PS2X ps2x;
    int _clk, _cmd, _att, _dat;

    uint16_t _buttons[MAX_BUTTONS] = {
        PSB_SELECT, PSB_L3, PSB_R3, PSB_START,
        PSB_PAD_UP, PSB_PAD_RIGHT, PSB_PAD_DOWN, PSB_PAD_LEFT,
        PSB_L2, PSB_R2, PSB_L1, PSB_R1,
        PSB_TRIANGLE, PSB_CIRCLE, PSB_CROSS, PSB_SQUARE
    };

    bool _currentState[MAX_BUTTONS];
    bool _lastState[MAX_BUTTONS];

public:
    PS2Driver(int clk, int cmd, int att, int dat);

    bool init();

    void update();

    bool pressed(uint16_t button);
    bool justPressed(uint16_t button);
    bool justReleased(uint16_t button);

private:
    int findIndex(uint16_t button);
};

#endif