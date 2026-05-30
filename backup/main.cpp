#include "StepperMotor.h"
#include "Relay.h"
#include "Config.h"
#include "PS2Driver.h"
#include "OledDisplay.h"
#include "DualMPU6050.h"
#include "RollController.h"
#include <TimerOne.h>

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

// MPU6050双传感器
DualMPU6050 mpu;

unsigned long speed = 750; //This is the pulse time interval (smaller,faster!)
unsigned long blockingSpeed = 3000; // 阻塞式步进的速度（默认较慢以保证可靠性）
constexpr unsigned long kMotorAnglePrintIntervalMs = 500;

// 定时器中断服务函数
void motorISR() {
    // 在中断中更新所有电机
    StepperMotor::updateAllInInterrupt();
}

void printSystemStatusTask() {
    static unsigned long lastPrintMs = 0;
    unsigned long now = millis();

    if (now - lastPrintMs < kMotorAnglePrintIntervalMs) {
        return;
    }

    lastPrintMs = now;

    // 打印电机步数
    Serial.print("Steps -> X: ");
    Serial.print(motorX.getCurrentPosition());
    Serial.print(", Y: ");
    Serial.print(motorY.getCurrentPosition());
    Serial.print(", Z: ");
    Serial.print(motorZ.getCurrentPosition());
    Serial.print(", A: ");
    Serial.print(motorA.getCurrentPosition());

    // 打印MPU6050 roll平均值
    Serial.print(" | Roll Avg: ");
    if (mpu.isReady()) {
        Serial.println(mpu.getAverageRoll(), 2); // 保留2位小数
    } else {
        Serial.println("N/A");
    }
}


void handlePS2Input() {
    StepperMotor* m = motors[currentMotor];
    bool driveEnabled = relay.isOn();
    bool motorCommandRejected = false;

    // --- 切换当前电机 ---
    if (ps2.justPressed(PSB_L1)) {
        StepperMotor* prevMotor = motors[currentMotor];
        currentMotor = (currentMotor + 1) % 4;
        prevMotor->stop();
        oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
        // Serial.print("Motor: ");
        // Serial.println(currentMotor);
    }

    if (ps2.justPressed(PSB_L2)) {
        StepperMotor* prevMotor = motors[currentMotor];
        currentMotor = (currentMotor - 1 + 4) % 4;
        prevMotor->stop();
        oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
        // Serial.print("Motor: ");
        // Serial.println(currentMotor);
    }

    // --- R1: 正转200步（阻塞式，一圈） ---
    if (ps2.justPressed(PSB_R1)) {
        if (driveEnabled && (!m->isContinuousMode() || !m->isRunning())) {
            // Serial.println("R1: CW 200 steps");
            m->stop();
            m->stepBlocking(200, blockingSpeed); //8000
            // Serial.println("R1: Done");
            oled.setMotorState(currentMotor, m->getCurrentPosition());
        } else if (!driveEnabled) {
            // Serial.println("R1 ignored: relay is OFF");
            motorCommandRejected = true;
        }
    }

    // --- R2: 反转200步（阻塞式，一圈） ---
    if (ps2.justPressed(PSB_R2)) {
        if (driveEnabled && (!m->isContinuousMode() || !m->isRunning())) {
            // Serial.println("R2: CCW 200 steps");
            m->stop();
            m->stepBlocking(-200, blockingSpeed);
            // Serial.println("R2: Done");
            oled.setMotorState(currentMotor, m->getCurrentPosition());
        } else if (!driveEnabled) {
            // Serial.println("R2 ignored: relay is OFF");
            motorCommandRejected = true;
        }
    }

    // --- 三角：持续正转 ---
    if (driveEnabled && ps2.pressed(PSB_TRIANGLE)) {
        m->run(CW, speed);
    }
    // --- 叉：持续反转 ---
    else if (driveEnabled && ps2.pressed(PSB_CROSS)) {
        m->run(CCW, speed);
    }
    else if (m->isContinuousMode()) {
        m->stop();
    }

    if (!driveEnabled && (ps2.pressed(PSB_TRIANGLE) || ps2.pressed(PSB_CROSS))) {
        motorCommandRejected = true;
    }

    if (driveEnabled && (ps2.justReleased(PSB_TRIANGLE) || ps2.justReleased(PSB_CROSS))) {
        oled.setMotorState(currentMotor, m->getCurrentPosition());
    }

    if (motorCommandRejected) {
        oled.showAccessDenied("Relay is off!", "Cmd rejected...");
    }

    // --- START: 切换继电器状态 ---
    if (ps2.justPressed(PSB_START)){
        relay.toggle();
        // if(relay.isOn())
        //     Serial.println("Toggle the relay to ON");
        // else
        //     Serial.println("Toggle the relay to OFF");
    }

    // --- CIRCLE: 重置所有电机位置为0，并清零传感器 ---
    if (ps2.justPressed(PSB_CIRCLE)) {
        for (int i = 0; i < 4; i++) {
            motors[i]->stop();           // 先停止电机运行
            motors[i]->resetPosition();  // 再重置位置为0
        }
        // 清零MPU6050传感器
        if (mpu.isReady()) {
            mpu.zero();
        }
        oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
        // Serial.println("All motor positions reset to 0, sensors zeroed");
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

    // 注册电机到中断系统
    StepperMotor::registerMotor(&motorX);
    StepperMotor::registerMotor(&motorY);
    StepperMotor::registerMotor(&motorZ);
    StepperMotor::registerMotor(&motorA);

    relay.begin();

    ps2.init();

    oled.begin();

    // 初始化MPU6050，最多等待10秒
    oled.showStatusText("Initializing", "MPU6050...");

    unsigned long startTime = millis();
    const unsigned long timeoutMs = 10000; // 10秒超时

    bool mpuInitialized = false;

    while (millis() - startTime < timeoutMs) {
        if (mpu.begin()) {
            mpuInitialized = true;
            break;
        }
        delay(100); // 短暂延迟后重试
    }

    if (mpuInitialized) {
        // 初始化成功，等待2秒让传感器稳定，然后自动清零
        delay(2000);
        if (mpu.isReady()) {
            mpu.zero();
            // Serial.println("MPU6050 sensors zeroed at startup");
        }
    } else {
        // MPU6050初始化失败，显示错误
        oled.showStatusText("MPU6050 Error", "Check connection");
        delay(3000); // 显示错误3秒
        // 继续启动，但MPU6050功能将不可用
    }

    oled.showBootScreen();
    delay(1500);
    oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
    oled.update();

    // Serial.println("System Ready");

    // 配置定时器中断，每50微秒触发一次
    Timer1.initialize(50);  // 50微秒周期
    Timer1.attachInterrupt(motorISR);
}


void loop() {

    /* // 测试RollController功能（只执行一次）
    static bool testExecuted = false;
    if (!testExecuted && relay.isOn() && mpu.isReady()) {
        // 确保当前电机不在连续模式
        StepperMotor* currentMotorPtr = motors[currentMotor];
        if (!currentMotorPtr->isContinuousMode() || !currentMotorPtr->isRunning()) {
            testExecuted = true;

            // 在OLED上显示测试状态
            oled.showStatusText("Roll Test:");
            oled.update();

            // 调用自动调整函数，目标为30度
            bool success = adjustRollToTarget(*currentMotorPtr, mpu, -20.0f, 0.3f, speed, 15000);

            if (success) {
                oled.showStatusText("Test Complete", "angle reached");
                // Serial.println("RollController test: Roll adjustment to 30° complete");
            } else {
                oled.showStatusText("Test Failed", "Timeout/Error");
                // Serial.println("RollController test: Roll adjustment failed or timeout");
            }

            delay(1000);  // 显示结果1秒

            // 恢复电机状态显示
            oled.setMotorState(currentMotor, currentMotorPtr->getCurrentPosition());
        }
    }
*/

    //读取ps2数据
    ps2.update();
    //处理ps2案件事件
    handlePS2Input();

    oled.update();
    printSystemStatusTask();

    // 更新MPU6050传感器数据
    mpu.update();

    // ===== 电机现在由定时器中断更新 =====
    // motorX.update();
    // motorY.update();
    // motorZ.update();
    // motorA.update();
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
