/*
  自动化检测序列控制器 — 实现
  ============================
  通过预定义的 SequenceStep 数组控制机器人的自动化运动流程。

  序列推进机制：
  - 自动模式（update）：每帧执行当前步骤 → 递增索引 → 下一帧执行下一步
  - 单步模式（executeOneStep）：手动触发执行当前步骤 → 递增索引
  - STEP_REPEAT 步骤：不递增索引，直接将索引重置为 0

  每步完成后自动调用 printMotorStates() 向串口输出当前电机状态，
  供上位机 serial_logger.py 采集记录。

  作者：ZENG Minyu
*/
#include "AutomationSequence.h"
#include <Arduino.h>

// 外部声明 — 这些常量在 main.cpp 中定义，供序列步骤使用
extern const long ARM_EXTEND_STEPS;    // 摇臂伸出步数（1600）
extern const long ARM_RETRACT_STEPS;   // 摇臂收回步数（-1600）
extern const long FORWARD_STEPS;       // 前进步数（100）
extern const long ROTATION_STEPS;      // 旋转步数（25）
extern const unsigned long DEFAULT_SPEED;      // 默认电机速度（5500 μs 间隔）
extern const float DEFAULT_TOLERANCE;          // 默认角度容差（0.5°）

// ============================================================
// 辅助宏：简化序列步骤的定义
// ============================================================

#define STEP_STEPS(motor, steps_val) \
    {STEP_MOTOR_STEPS, motor, {.steps = steps_val}, DEFAULT_SPEED, DEFAULT_TOLERANCE}

#define STEP_WAIT(ms_val) \
    {STEP_WAIT_MS, 0, {.waitMs = ms_val}, 0, DEFAULT_TOLERANCE}

#define STEP_EXTEND(motor) \
    {STEP_MOTOR_EXTEND, motor, {.steps = 0}, DEFAULT_SPEED, DEFAULT_TOLERANCE}

#define STEP_RETRACT(motor) \
    {STEP_MOTOR_RETRACT, motor, {.steps = 0}, DEFAULT_SPEED, DEFAULT_TOLERANCE}

#define STEP_REPEAT() \
    {STEP_REPEAT, 0, {.steps = 0}, 0, DEFAULT_TOLERANCE}

// ============================================================
// 默认检测序列（当前生效版本）
// ============================================================
// 序列逻辑：重复 5 轮，每轮"前进 100 步 → 旋转扫描（逆时针 50 + 4×30 = 170 步）
//           → 顺时针归位 120 步"
// 总计覆盖 5 个轴向位置 × 约 19.4° 的旋转扫描扇区
//
// 注意：以下为简化测试版本。完整版（注释区块）包含摇臂伸出/收回 + 超声波采集等待，
// 因缺少径向限位开关而暂未启用。

const SequenceStep defaultSequence[] = {

    // --- 第 1 轮 ---
    STEP_STEPS(3, 100),      // Motor A (驱动轮) 前进 100 步
    STEP_STEPS(1, -50),      // Motor Y 逆时针 50 步（快速回扫起点）
    STEP_STEPS(1, 30),       // Motor Y 顺时针 30 步扫描（4 次 × 30 = 120 步扫描）
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, -120),     // Motor Y 逆时针归位 120 步

    // --- 第 2 轮 ---
    STEP_STEPS(3, 100),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, -120),

    // --- 第 3 轮 ---
    STEP_STEPS(3, 100),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, -120),

    // --- 第 4 轮 ---
    STEP_STEPS(3, 100),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, -120),

    // --- 第 5 轮 ---
    STEP_STEPS(3, 100),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, 30),
    STEP_STEPS(1, -120),

// ============================================================
// 完整版检测序列（注释中，待限位开关就绪后启用）
// ============================================================
/*
    // --- 第 1 轮 ---
    STEP_STEPS(3, FORWARD_STEPS),   // 前进
    STEP_EXTEND(0),                 // 伸出扫描臂
    STEP_WAIT(1000),                // 等待超声波采集
    STEP_RETRACT(0),                // 收回扫描臂
    STEP_STEPS(1, ROTATION_STEPS),  // 旋转一个角度步进

    // --- 第 2 轮 ---
    STEP_STEPS(3, FORWARD_STEPS),
    STEP_EXTEND(0),
    STEP_WAIT(1000),
    STEP_RETRACT(0),
    STEP_STEPS(1, ROTATION_STEPS),
*/
};

// ============================================================
// 构造函数
// ============================================================

AutomationSequence::AutomationSequence(StepperMotor** motors)
    : _motors(motors), _sequence(nullptr), _sequenceLength(0),
      _currentStepIndex(0), _running(false) {
}

// ============================================================
// 序列生命周期
// ============================================================

void AutomationSequence::start() {
    if (_running) return;  // 防止重复启动

    _running = true;
    _currentStepIndex = 0;

    // 自动加载默认序列（如果尚未通过 setDefaultSequence 加载）
    if (_sequence == nullptr) {
        setDefaultSequence();
    }
}

// ============================================================
// 自动模式更新（每帧由 loop() 调用）
// ============================================================

void AutomationSequence::update() {
    if (!_running) return;

    // 序列完成：索引超出数组末尾
    if (_currentStepIndex >= _sequenceLength) {
        _running = false;
        return;
    }

    SequenceStepType currentType = _sequence[_currentStepIndex].type;

    // 执行当前步骤（阻塞式，步骤完成才返回）
    executeStep(_sequence[_currentStepIndex]);

    // REPEAT 步骤已在 executeStep 中将索引重置为 0，不需要额外递增
    if (currentType != STEP_REPEAT) {
        _currentStepIndex++;
    }
}

// ============================================================
// 单步执行（PS2 ↑ 键触发，与自动运行互不干扰）
// ============================================================

void AutomationSequence::executeOneStep() {
    // 自动加载默认序列
    if (_sequence == nullptr) {
        setDefaultSequence();
    }

    // 到达末尾则回到开头（循环）
    if (_currentStepIndex >= _sequenceLength) {
        _currentStepIndex = 0;
    }

    SequenceStepType currentType = _sequence[_currentStepIndex].type;
    executeStep(_sequence[_currentStepIndex]);
    if (currentType != STEP_REPEAT) {
        _currentStepIndex++;
    }
}

// ============================================================
// 步骤执行（switch 分发 + 串口输出）
// ============================================================

void AutomationSequence::executeStep(const SequenceStep& step) {
    switch (step.type) {
        case STEP_MOTOR_STEPS:
            // 最常用：调用 stepBlocking 精确走完指定步数
            _motors[step.motorIndex]->stepBlocking(step.params.steps, step.speed, nullptr);
            break;

        case STEP_MOTOR_ANGLE:
            // 预留：需配合角度传感器将角度转换为步数（当前跳过不执行）
            break;

        case STEP_WAIT_MS:
            {
                // 分片 delay：每 10ms 检查一次，避免长时间阻塞
                unsigned long start = millis();
                while (millis() - start < step.params.waitMs) {
                    delay(10);
                }
            }
            break;

        case STEP_MOTOR_EXTEND:
            // 伸出扫描臂（使用 main.cpp 中定义的 ARM_EXTEND_STEPS 常量）
            _motors[step.motorIndex]->stepBlocking(ARM_EXTEND_STEPS, step.speed, nullptr);
            break;

        case STEP_MOTOR_RETRACT:
            // 收回扫描臂（使用 main.cpp 中定义的 ARM_RETRACT_STEPS 常量，负值）
            _motors[step.motorIndex]->stepBlocking(ARM_RETRACT_STEPS, step.speed, nullptr);
            break;

        case STEP_REPEAT:
            // 重置索引到开头，实现序列循环
            _currentStepIndex = 0;
            return;  // 不执行后续的 printMotorStates()
    }
    // 每步完成后向串口输出当前状态，供上位机记录
    printMotorStates();
}

// ============================================================
// 串口状态输出
// ============================================================

void AutomationSequence::printMotorStates() const {
    Serial.print("X: ");
    Serial.print(_motors[0]->getCurrentPosition());
    Serial.print(", Y: ");
    Serial.print(_motors[1]->getCurrentPosition());
    Serial.print(", Z: ");
    Serial.print(_motors[2]->getCurrentPosition());
    Serial.print(", A: ");
    Serial.println(_motors[3]->getCurrentPosition());
}

// ============================================================
// 加载默认序列
// ============================================================

void AutomationSequence::setDefaultSequence() {
    _sequence = defaultSequence;
    _sequenceLength = sizeof(defaultSequence) / sizeof(defaultSequence[0]);
}
