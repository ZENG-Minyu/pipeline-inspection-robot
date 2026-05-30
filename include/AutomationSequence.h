/*
  自动化检测序列控制器
  ====================
  通过预定义的步骤序列实现管道检测的自动化运动流程。
  由 PS2 ■ 键触发自动连续执行，或由 PS2 ↑ 键单步调试。

  序列步骤类型：
  - STEP_MOTOR_STEPS：指定电机移动指定步数（正=前进/CW，负=后退/CCW）
  - STEP_MOTOR_ANGLE：按角度旋转（预留，暂未实现）
  - STEP_WAIT_MS：等待指定毫秒数（用于数据采集延时）
  - STEP_MOTOR_EXTEND：伸出扫描臂（= +ARM_EXTEND_STEPS）
  - STEP_MOTOR_RETRACT：收回扫描臂（= -ARM_RETRACT_STEPS）
  - STEP_REPEAT：跳回序列开头，实现循环

  当前生效的默认序列（简化版）：
  重复 5 次：
    1. Motor A (驱动轮): 前进 100 步
    2. Motor Y (旋转盘): 逆时针扫描 50 + 4×30 = 170 步，顺时针归位 120 步

  完整版序列（注释中）：包含摇臂伸出 + 超声波采集等待 + 摇臂收回，
  因缺少径向限位开关而暂未启用。

  触发方式：
  - PS2 ■ 键：加载默认序列并自动连续执行（通过 update() 推进）
  - PS2 ↑ 键：执行单步（executeOneStep），不依赖自动运行状态

  作者：ZENG Minyu
*/
#ifndef AUTOMATION_SEQUENCE_H
#define AUTOMATION_SEQUENCE_H

#include <Arduino.h>
#include "StepperMotor.h"

// 序列步骤类型枚举
enum SequenceStepType {
    STEP_MOTOR_STEPS,      // 电机移动指定步数（最常用的步骤类型）
    STEP_MOTOR_ANGLE,      // 电机旋转到目标角度（预留，需配合角度传感器）
    STEP_WAIT_MS,          // 等待指定毫秒数（用于传感器采集延时）
    STEP_MOTOR_EXTEND,     // 伸出摇臂（使用预定义的 ARM_EXTEND_STEPS 常量）
    STEP_MOTOR_RETRACT,    // 收回摇臂（使用预定义的 ARM_RETRACT_STEPS 常量）
    STEP_REPEAT            // 重复序列：跳回步骤 0，实现无限循环
};

// 序列步骤定义结构体
// 使用 union 节约内存，同一时刻仅使用 params 中的一个字段
struct SequenceStep {
    SequenceStepType type;     // 步骤类型
    uint8_t motorIndex;        // 目标电机索引 (0=X, 1=Y, 2=Z, 3=A)
    union {
        long steps;            // 步数（正=正转，负=反转），用于 STEP_MOTOR_STEPS
        float angle;           // 目标角度（度），用于 STEP_MOTOR_ANGLE（预留）
        unsigned long waitMs;  // 等待时间（毫秒），用于 STEP_WAIT_MS
    } params;
    unsigned long speed;       // 电机运转速度（微秒脉冲间隔）
    float tolerance;           // 角度容差（度），用于 STEP_MOTOR_ANGLE（预留）
};

class AutomationSequence {
public:
    // 构造函数：传入 motors 指针数组，用于在步骤执行时操作对应电机
    AutomationSequence(StepperMotor** motors);

    // 启动序列：设置 _running = true，从步骤 0 开始执行
    void start();

    // 主循环更新：每帧调用，执行当前步骤并推进到下一步
    void update();

    // 查询序列是否正在自动运行
    bool isRunning() const { return _running; }

    // 获取当前步骤索引
    uint8_t getCurrentStep() const { return _currentStepIndex; }

    // 单步执行：执行当前步骤并前进到下一步
    // 不依赖 _running 标志，与 update() 互不干扰
    // 用于 PS2 ↑ 键的手动调试模式
    void executeOneStep();

    // 向串口打印四个电机的当前步数（每步完成后自动调用）
    void printMotorStates() const;

    // 加载预定义的默认检测序列
    void setDefaultSequence();

private:
    // 执行单个步骤的具体逻辑（switch 分发）
    void executeStep(const SequenceStep& step);

    StepperMotor** _motors;          // 电机指针数组（由 main.cpp 传入）

    const SequenceStep* _sequence;   // 当前使用的序列数组
    size_t _sequenceLength;          // 序列数组长度（步骤总数）
    size_t _currentStepIndex;        // 当前执行到的步骤索引

    bool _running;                   // 自动运行状态标志
};

#endif // AUTOMATION_SEQUENCE_H
