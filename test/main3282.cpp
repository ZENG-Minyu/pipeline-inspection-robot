#include "StepperMotor.h"
#include "Relay.h"
#include "Config.h"
#include "PS2Driver.h"
#include "OledDisplay.h"

// 电机
StepperMotor motorX(PIN_STEP_X, PIN_DIR_X);
StepperMotor motorY(PIN_STEP_Y, PIN_DIR_Y);
StepperMotor motorZ(PIN_STEP_Z, PIN_DIR_Z);
StepperMotor motorA(PIN_STEP_A, PIN_DIR_A);

// 数组管理
StepperMotor* motors[4] = {&motorX, &motorY, &motorZ, &motorA};
int currentMotor = 0;

// PS2
PS2Driver ps2(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);

// 继电器
Relay relay(52);

// OLED
OledDisplay oled;

unsigned long speed = 750;
unsigned long displayRefreshMs = 100;

void updateDisplayTask() {
    static unsigned long lastRefresh = 0;
    static int lastMotor = -1;
    static long lastPosition = 0x7fffffff;

    if (millis() - lastRefresh < displayRefreshMs) {
        return;
    }
    lastRefresh = millis();

    StepperMotor* m = motors[currentMotor];
    long currentPosition = m->getCurrentPosition();

    if (currentMotor == lastMotor && currentPosition == lastPosition) {
        return;
    }

    lastMotor = currentMotor;
    lastPosition = currentPosition;

    char motorLine[16];
    char positionLine[20];

    snprintf(motorLine, sizeof(motorLine), "Motor: %d", currentMotor);
    snprintf(positionLine, sizeof(positionLine), "Pos: %ld", currentPosition);

    oled.showPresetText(motorLine, positionLine);
}


void handlePS2Input() {
    StepperMotor* m = motors[currentMotor];

    // --- 切换当前电机 ---
    if (ps2.justPressed(PSB_L1)) {
        currentMotor = (currentMotor + 1) % 4;
        m->stop();
        Serial.print("Motor: ");
        Serial.println(currentMotor);
    }

    if (ps2.justPressed(PSB_L2)) {
        currentMotor = (currentMotor - 1 + 4) % 4;
        m->stop();
        Serial.print("Motor: ");
        Serial.println(currentMotor);
    }

        // --- R1: 正转200步（阻塞式，一圈） ---
    if (ps2.justPressed(PSB_R1)) {
        if (!m->isContinuousMode() || !m->isRunning()) {
            Serial.println("R1: CW 200 steps");
            m->stop();
            m->stepBlocking(200, speed); //8000
            Serial.println("R1: Done");
        }
    }

    // --- R2: 反转200步（阻塞式，一圈） ---
    if (ps2.justPressed(PSB_R2)) {
        if (!m->isContinuousMode() || !m->isRunning()) {
            Serial.println("R2: CCW 200 steps");
            m->stop();
            m->stepBlocking(-200, speed);
            Serial.println("R2: Done");
        }
    }

    // --- 三角：持续正转 ---
    if (ps2.pressed(PSB_TRIANGLE)) {
        m->run(CW, speed);
    }
    // --- 叉：持续反转 ---
    else if (ps2.pressed(PSB_CROSS)) {
        m->run(CCW, speed);
    }
    else {
        if (m->isContinuousMode()) {
            m->stop();
        }
    }

    // --- START: 切换继电器状态 ---
    if (ps2.justPressed(PSB_START)){
        relay.toggle();
        if(relay.isOn())
            Serial.println("Toggle the relay to ON");
        else
            Serial.println("Toggle the relay to OFF");
    }
}


void setup() {
    Serial.begin(115200);

    motorX.begin();
    motorY.begin();
    motorZ.begin();
    motorA.begin();

    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, LOW);

    relay.begin();

    ps2.init();

    oled.begin();
    oled.showSystemReady();

    Serial.println("System Ready");


}


void loop() {

    //读取ps2数据
    ps2.update();
    //处理ps2案件事件
    handlePS2Input();

    updateDisplayTask();

    // ===== 更新电机 =====
    motorX.update();
    motorY.update();
    motorZ.update();
    motorA.update();
}


//这段代码用来检测主循环频率
// void loop() {
//     static unsigned long lastPrint = 0;
//     static int count = 0;
//     count++;
//     if (millis() - lastPrint >= 1000) {
//         Serial.print("Loop freq: ");
//         Serial.print(count);
//         Serial.println(" Hz");
//         count = 0;
//         lastPrint = millis();
//     }
//     ps2.update();
//     handlePS2Input();
//     motorX.update();
//     motorY.update();
//     motorZ.update();
//     motorA.update();
// }
