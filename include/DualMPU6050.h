/*
  双 MPU6050 姿态传感器驱动（已弃用 / Deprecated）
  ==============================================
  设计初衷：通过两个 MPU6050（I2C 地址 0x68 和 0x69）分别采集 Roll 角度，
           取平均值来实现旋转圆盘的闭环角度控制。

  弃用原因：
  1. I2C 信号需从前段经过约 1m 长线缆传输到后段，信号衰减严重
  2. 12V 电机 PWM 电路产生强电磁干扰（EMI），进一步恶化 I2C 通信稳定性
  3. 传感器读数频繁失效（连接断开、数据校验失败），无法可靠用于闭环控制

  替代方案：改为开环步数计数 —— 利用齿轮减速比和内齿圈传动关系，
           从 Y 电机步数直接推算旋转角度（2230 steps = 360°）。

  代码状态：本文件及其对应 .cpp 保留在项目中供参考，但 main.cpp 中已不再引用。
           相关依赖库（lib/MPU6050、lib/I2Cdev）也仅为此模块服务。

  作者：ZENG Minyu
*/
#ifndef DUAL_MPU6050_H
#define DUAL_MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class DualMPU6050 {
public:
    DualMPU6050();

    // 初始化两个 MPU6050 传感器（I2C 地址 0x68 和 0x69）
    // 返回 true 表示两个传感器均成功初始化并通过连接测试
    bool begin();

    // 非阻塞更新：从两个传感器的 DMP FIFO 读取最新的姿态数据
    // 应在主循环中高频调用（每次 loop 调用一次）
    void update();

    // 获取两个传感器 Roll 角度的算术平均值（单位：度）
    float getAverageRoll() const;

    // 检查两个传感器是否均已成功初始化且 DMP 就绪
    bool isReady() const;

    // 检查初始化阶段是否发生错误
    bool hasError() const;

    // 运行时连接状态检查：对两个传感器分别调用 testConnection()
    bool isConnected();

    // 分别获取传感器 A (0x68) 和 B (0x69) 的 Roll 角度（度）
    float getRollA() const;
    float getRollB() const;

    // 将当前 Roll 角度记录为偏移量，后续读数减去此偏移量（软件归零）
    void zero();

private:
    MPU6050_6Axis_MotionApps20 mpuA;  // I2C 地址 0x68
    MPU6050_6Axis_MotionApps20 mpuB;  // I2C 地址 0x69

    bool dmpReadyA;            // 传感器 A 的 DMP 是否就绪
    bool dmpReadyB;            // 传感器 B 的 DMP 是否就绪
    bool initializationError;  // 初始化是否失败

    float yprA[3];             // 传感器 A 的 [Yaw, Pitch, Roll]（弧度）
    float yprB[3];             // 传感器 B 的 [Yaw, Pitch, Roll]（弧度）

    float rollOffsetA;         // 传感器 A 的 Roll 偏移量（度，用于归零）
    float rollOffsetB;         // 传感器 B 的 Roll 偏移量（度）

    uint8_t fifoBuffer[64];    // DMP FIFO 数据包缓冲区（复用，每次读取一个传感器）

    Quaternion q;              // 四元数临时变量（复用）
    VectorFloat gravity;       // 重力向量临时变量（复用）

    // 初始化单个 MPU6050：连接测试 → DMP 初始化 → 加速度计/陀螺仪校准 → 启用 DMP
    bool initMPU(MPU6050_6Axis_MotionApps20& mpu, uint8_t address, const char* label);
};

#endif
