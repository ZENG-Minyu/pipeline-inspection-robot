/*
  旋转角度闭环控制器 — 实现（已弃用 / Deprecated）
  ================================================
  因 DualMPU6050 不可靠而弃用。当前为占位实现，直接返回 true。

  作者：ZENG Minyu
*/
#include "RollController.h"

bool adjustRollToTarget(StepperMotor& motor, DualMPU6050& mpu,
                       float targetRollDeg,
                       float toleranceDeg,
                       unsigned long motorSpeed,
                       unsigned long timeoutMs,
                       bool (*shouldContinue)()) {
    // 已弃用：因 MPU6050 传感器不可靠，闭环控制无法正常工作
    // 替代方案见 main.cpp 中的开环步数计数方案
    return true;
}
