/*
  管道检测机器人 — 主程序入口
  ============================
  硬件平台：Arduino Mega 2560 (ATmega2560, 16MHz)
  扩展板：CNC Shield v3 + 4× A4988 步进电机驱动器
  固件框架：Arduino (PlatformIO 编译链)

  系统架构概述：
  ┌─────────────────────────────────────────────────┐
  │  上位机 (Host PC)                                │
  │  python/serial_logger.py → 串口数据采集          │
  │  python/log_to_coords.py → 步数→柱坐标转换       │
  └───────────────┬─────────────────────────────────┘
                  │ USB Serial 115200 baud
  ┌───────────────▼─────────────────────────────────┐
  │  下位机主控 (Arduino Mega 2560)                  │
  │  - PS2 手柄遥控 (SPI 软件模拟)                    │
  │  - OLED 状态显示 (I2C SSD1306 128×64)             │
  │  - 继电器 12V 电源控制 (D52, Active LOW)          │
  │  - 4× 步进电机 (X/Y/Z/A, 通过 CNC Shield v3)     │
  └─────────────────────────────────────────────────┘

  电机功能映射：
  - Motor 0 (X 轴): 探头径向伸缩（扫描臂升降丝杠）
  - Motor 1 (Y 轴): 360° 旋转扫描（旋转圆盘，小齿轮→内齿圈，2230 steps/rev）
  - Motor 2 (Z 轴): 预留
  - Motor 3 (A 轴): 整机前进/后退（3 轮同步驱动，1/8 微步，接线端并联）

  定时器中断架构：
  - Timer1 配置为 100μs 周期中断（10kHz）
  - ISR 中顺序调用 4 个电机的 updateDirectInInterrupt()
  - 将脉冲时序精度从主循环的 ~3ms 提升至 ~100μs，彻底解决共振失步问题

  作者：ZENG Minyu
*/
#include "StepperMotor.h"
#include "Relay.h"
#include "Config.h"
#include "PS2Driver.h"
#include "OledDisplay.h"
#include "AutomationSequence.h"
#include <TimerOne.h>

// ============================================================
// 全局对象实例化
// ============================================================

// 四个步进电机 — 引脚定义来自 Config.h
StepperMotor motorX(PIN_STEP_X, PIN_DIR_X);  // X 轴：径向伸缩
StepperMotor motorY(PIN_STEP_Y, PIN_DIR_Y);  // Y 轴：旋转扫描
StepperMotor motorZ(PIN_STEP_Z, PIN_DIR_Z);  // Z 轴：预留
StepperMotor motorA(PIN_STEP_A, PIN_DIR_A);  // A 轴：前进/后退

// 电机指针数组 — 方便通过索引循环操作
StepperMotor* motors[4] = {&motorX, &motorY, &motorZ, &motorA};
int currentMotor = 0;  // 当前选中的电机索引 (PS2 L1/L2 切换)

// PS2 无线手柄 — SPI 引脚定义来自 Config.h
PS2Driver ps2(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);

// 继电器 — 控制 12V 动力电源通断（D52）
Relay relay(52);

// OLED 显示屏 — SSD1306 128×64 I2C
OledDisplay oled;

// 自动化序列控制器 — 传入电机数组供步骤执行
AutomationSequence sequence(motors);

// ============================================================
// 全局参数
// ============================================================

// 连续运转模式速度（PS2 ▲/✕ 按住时的脉冲间隔，μs）
// 值越小速度越快，但需考虑电机的启动扭矩限制
unsigned long speed = 4000;

// 阻塞式步进速度（PS2 R1/R2 点动和自动化序列使用的脉冲间隔，μs）
// 默认较慢（6000μs ≈ 167Hz），确保带载可靠运行
unsigned long blockingSpeed = 6000;

// 串口打印电机状态的间隔（毫秒）
constexpr unsigned long kMotorAnglePrintIntervalMs = 500;

// ============================================================
// 自动化序列参数常量
// ============================================================
// 这些常量在 AutomationSequence.cpp 中通过 extern 引用

extern const long ARM_EXTEND_STEPS = 1600;     // 摇臂伸出步数（全行程）
extern const long ARM_RETRACT_STEPS = -1600;   // 摇臂收回步数（负值 = 反向）
extern const long FORWARD_STEPS = 100;         // 每轮前进步数（约 13.7mm）
extern const long ROTATION_STEPS = 25;         // 每次旋转步数（约 4°）
extern const unsigned long DEFAULT_SPEED = 5500;       // 默认电机速度（μs 间隔）
extern const float DEFAULT_TOLERANCE = 0.5f;           // 默认角度容差（度，预留）

// PS2 R1/R2 点动步数映射表（按电机索引 X/Y/Z/A）
const long kR1R2Steps[4] = {800, ROTATION_STEPS, 200, FORWARD_STEPS};
// Motor 0 (X): ±800 步 — 摇臂半行程伸缩
// Motor 1 (Y): ±25 步 — 旋转约 4°
// Motor 2 (Z): ±200 步 — 预留
// Motor 3 (A): ±100 步 — 前进/后退约 13.7mm

// ============================================================
// 定时器中断服务函数 (ISR)
// ============================================================
// 由 Timer1 每 100μs 调用一次（10kHz 中断频率）。
// 顺序调用四个电机的 updateDirectInInterrupt()，在 ISR 中直接生成步进脉冲。
//
// 设计原因：
// - 主循环包含 PS2 通信、OLED I2C 传输、串口输出等耗时操作，循环频率波动大
//   （实测约 330Hz = 3ms 周期）
// - 电机步进需要微秒级精度的脉冲间隔——例如 5500μs 的脉冲间隔要求 ±100μs
//   的精度才能避免抖动
// - 100μs 定时器中断提供比主循环高两个数量级的时间分辨率
// - 每次 ISR 执行时间在纳秒级，不影响主循环
void motorISR() {
    motorX.updateDirectInInterrupt();
    motorY.updateDirectInInterrupt();
    motorZ.updateDirectInInterrupt();
    motorA.updateDirectInInterrupt();
}

// ============================================================
// 定时串口状态输出（当前未启用，通过 printSystemStatusTask 调用）
// ============================================================

void printSystemStatusTask() {
    static unsigned long lastPrintMs = 0;
    unsigned long now = millis();

    if (now - lastPrintMs < kMotorAnglePrintIntervalMs) {
        return;
    }

    lastPrintMs = now;

    Serial.print("Steps -> X: ");
    Serial.print(motorX.getCurrentPosition());
    Serial.print(", Y: ");
    Serial.print(motorY.getCurrentPosition());
    Serial.print(", Z: ");
    Serial.print(motorZ.getCurrentPosition());
    Serial.print(", A: ");
    Serial.println(motorA.getCurrentPosition());
}

// ============================================================
// PS2 手柄按键处理
// ============================================================

void handlePS2Input() {
    StepperMotor* m = motors[currentMotor];
    bool driveEnabled = relay.isOn();       // 继电器状态 = 电机供电是否就绪
    bool motorCommandRejected = false;      // 是否有指令因继电器断开被拒绝

    // --- L1/R1 切换当前选中电机 ---
    // L1：切换到下一个电机（索引 +1）
    if (ps2.justPressed(PSB_L1)) {
        StepperMotor* prevMotor = motors[currentMotor];
        currentMotor = (currentMotor + 1) % 4;
        prevMotor->stop();  // 停止上一个电机的运转
        oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
    }

    // L2：切换到上一个电机（索引 -1）
    if (ps2.justPressed(PSB_L2)) {
        StepperMotor* prevMotor = motors[currentMotor];
        currentMotor = (currentMotor - 1 + 4) % 4;
        prevMotor->stop();
        oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
    }

    // --- R1: 正向点动（步数因选中电机而异） ---
    if (ps2.justPressed(PSB_R1)) {
        if (driveEnabled && (!m->isContinuousMode() || !m->isRunning())) {
            long steps = kR1R2Steps[currentMotor];
            m->stop();
            m->stepBlocking(steps, blockingSpeed, nullptr);  // 阻塞式执行，完成才返回
            oled.setMotorState(currentMotor, m->getCurrentPosition());
        } else if (!driveEnabled) {
            motorCommandRejected = true;
        }
    }

    // --- R2: 反向点动 ---
    if (ps2.justPressed(PSB_R2)) {
        if (driveEnabled && (!m->isContinuousMode() || !m->isRunning())) {
            long steps = kR1R2Steps[currentMotor];
            m->stop();
            m->stepBlocking(-steps, blockingSpeed, nullptr);  // 负步数 = 反向
            oled.setMotorState(currentMotor, m->getCurrentPosition());
        } else if (!driveEnabled) {
            motorCommandRejected = true;
        }
    }

    // --- ▲ (三角): 持续正转（按住不放） ---
    if (driveEnabled && ps2.pressed(PSB_TRIANGLE)) {
        m->run(CW, speed);
    }
    // --- ✕ (叉): 持续反转（按住不放） ---
    else if (driveEnabled && ps2.pressed(PSB_CROSS)) {
        m->run(CCW, speed);
    }
    // 松手即停：三角和叉都未按下且当前在连续模式则停止
    else if (m->isContinuousMode()) {
        m->stop();
    }

    // 拒绝在继电器断开时执行持续运转
    if (!driveEnabled && (ps2.pressed(PSB_TRIANGLE) || ps2.pressed(PSB_CROSS))) {
        motorCommandRejected = true;
    }

    // 松开三角或叉时刷新 OLED 显示（更新最终步数）
    if (driveEnabled && (ps2.justReleased(PSB_TRIANGLE) || ps2.justReleased(PSB_CROSS))) {
        oled.setMotorState(currentMotor, m->getCurrentPosition());
    }

    // 显示"拒绝访问"警告（继电器断开时的任何电机指令被拒绝）
    if (motorCommandRejected) {
        oled.showAccessDenied("Relay is off!", "Cmd rejected...");
    }

    // --- ↑ (方向键上): 手动单步执行自动化序列（调试用） ---
    if (ps2.justPressed(PSB_PAD_UP)) {
        if (relay.isOn()) {
            if (!sequence.isRunning()) {
                int savedMotor = currentMotor;
                sequence.executeOneStep();  // 执行一步（不依赖自动运行状态）
                currentMotor = savedMotor;  // 恢复序列执行前选中的电机
                oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
            }
        } else {
            oled.showAccessDenied("Relay is off!", "Cmd rejected");
        }
    }

    // --- SELECT: 手动打印当前四轴步数到串口 ---
    if (ps2.justPressed(PSB_SELECT)) {
        sequence.printMotorStates();
    }

    // --- START: 切换继电器通断（12V 动力电源开关） ---
    if (ps2.justPressed(PSB_START)){
        relay.toggle();
    }

    // --- ○ (圆圈): 重置全部四轴步数计数器为 0 ---
    if (ps2.justPressed(PSB_CIRCLE)) {
        for (int i = 0; i < 4; i++) {
            motors[i]->stop();           // 先停止电机
            motors[i]->resetPosition();  // 再归零计数器（中断安全）
        }
        oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
    }

    // --- ■ (方块): 启动自动化检测序列 ---
    if (ps2.justPressed(PSB_SQUARE)) {
        if (relay.isOn()) {
            if (!sequence.isRunning()) {
                sequence.setDefaultSequence();
                sequence.start();
                oled.showStatusText("Sequence", "Started");
            }
            // 序列已运行时忽略重复按键
        } else {
            oled.showAccessDenied("Relay is off!", "Sequence blocked");
        }
    }
}

// ============================================================
// 系统初始化
// ============================================================

void setup() {
    // 1. 初始化串口通信（与上位机通信，115200 baud）
    Serial.begin(115200);

    // 2. 初始化四个步进电机引脚
    motorX.begin();
    motorY.begin();
    motorZ.begin();
    motorA.begin();

    // 3. 使能 CNC Shield（EN 引脚 LOW = 全部四轴使能）
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, LOW);

    // 4. 初始化继电器（默认断开，确保上电安全）
    relay.begin();

    // 5. 初始化 PS2 手柄（SPI 配对）
    ps2.init();

    // 6. 初始化 OLED 显示屏
    oled.begin();

    // 7. 显示启动画面并延迟 1.5 秒
    oled.showBootScreen();
    delay(1500);
    oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
    oled.update();

    // 8. 配置 Timer1 定时器中断
    //    周期 = 100μs，频率 = 10kHz
    //    绑定 motorISR 为中断服务函数
    Timer1.initialize(100);
    Timer1.attachInterrupt(motorISR);
}

// ============================================================
// 主循环
// ============================================================

void loop() {
    // 1. 读取 PS2 手柄最新状态（更新位掩码）
    ps2.update();

    // 2. 处理 PS2 手柄按键事件
    handlePS2Input();

    // 3. 刷新 OLED 显示
    oled.update();

    // 4. （可选）定时打印电机状态到串口 — 当前注释以节省串口带宽
    // printSystemStatusTask();

    // 5. 推进自动化序列状态机（每帧检查并执行一步）
    sequence.update();

    // 6. 检测序列状态变化并输出串口标记
    //    上位机 serial_logger.py 通过检测这些标记区分手动控制和自动序列
    static bool wasSequenceRunning = false;
    bool isSequenceRunning = sequence.isRunning();
    if (isSequenceRunning && !wasSequenceRunning) {
        // 序列刚刚启动
        Serial.println("=== Automation Sequence Started ===");
    } else if (!isSequenceRunning && wasSequenceRunning) {
        // 序列刚刚结束
        Serial.println("=== Automation Sequence Finished ===");
        // 恢复 OLED 正常显示（序列运行时 OLED 被步骤间快速刷新覆盖）
        oled.setMotorState(currentMotor, motors[currentMotor]->getCurrentPosition());
    }
    wasSequenceRunning = isSequenceRunning;

    // ===== 电机脉冲现在由定时器中断生成，不在此处调用 update() =====
    // motorX.update();
    // motorY.update();
    // motorZ.update();
    // motorA.update();
}

// ============================================================
// 附录：主循环频率测量代码（调试用，保留供参考）
// ============================================================
// 用于测量 loop() 的实际执行频率。在主循环中累加计数器，
// 每秒输出一次，从而判断主循环是否因耗时操作而降速。
//
// 实测结果（含 PS2 通信 + OLED 刷新）：约 330 Hz（3ms 周期）
//
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
