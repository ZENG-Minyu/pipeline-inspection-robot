#include "StepperMotor.h"
#include "Relay.h"
#include "Config.h"

// 电机实例
StepperMotor motorX(PIN_STEP_X, PIN_DIR_X);
StepperMotor motorY(PIN_STEP_Y, PIN_DIR_Y);
StepperMotor motorZ(PIN_STEP_Z, PIN_DIR_Z);
StepperMotor motorA(PIN_STEP_A, PIN_DIR_A);

// 继电器
Relay relay(52);

bool started = false;

void setup() {
    Serial.begin(115200);

    motorX.begin();
    motorY.begin();
    motorZ.begin();
    motorA.begin();

    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, LOW);

    relay.begin();

    if (!relay.isOn()) {
        Serial.println("Switching on the relay.");
        relay.toggle();
        delay(500);
    } else {
        Serial.println("Relay already on. Switching off.");
        relay.toggle();
        delay(500);
    }
}

void loop() {

    #define interval 1000

    // 🚀 启动一次测试
    if (!started) {
        Serial.println("Start moving motors...");

        motorX.moveTo(1000, interval);
        motorY.moveTo(800, interval);
        motorZ.moveTo(-600, interval);
        motorA.moveTo(400, interval);

        started = true;
    }

    // 🔁 非阻塞更新（核心！！）
    motorX.update();
    motorY.update();
    motorZ.update();
    motorA.update();

    // 所有电机完成后打印一次
    if (started &&
        !motorX.isRunning() &&
        !motorY.isRunning() &&
        !motorZ.isRunning() &&
        !motorA.isRunning()) {

        Serial.println("All motors finished!");

        Serial.print("X: "); Serial.println(motorX.getCurrentPosition());
        Serial.print("Y: "); Serial.println(motorY.getCurrentPosition());
        Serial.print("Z: "); Serial.println(motorZ.getCurrentPosition());
        Serial.print("A: "); Serial.println(motorA.getCurrentPosition());

        started = false;

        delay(2000); // 仅用于演示
    }
}