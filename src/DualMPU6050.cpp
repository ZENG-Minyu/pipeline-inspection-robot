/*
  双 MPU6050 姿态传感器驱动 — 实现（已弃用 / Deprecated）
  ======================================================
  本文件与 DualMPU6050.h 配套，因 I2C 长距离信号衰减和电机 EMI 干扰已弃用。
  详见头文件的弃用说明。

  作者：ZENG Minyu
*/
#include "DualMPU6050.h"

// ============================================================
// 构造函数
// ============================================================

DualMPU6050::DualMPU6050()
    : mpuA(0x68), mpuB(0x69),
      dmpReadyA(false), dmpReadyB(false),
      initializationError(false),
      rollOffsetA(0.0f), rollOffsetB(0.0f) {
    for (int i = 0; i < 3; i++) {
        yprA[i] = 0.0f;
        yprB[i] = 0.0f;
    }
}

// ============================================================
// 初始化
// ============================================================

bool DualMPU6050::begin() {
    Wire.begin();
    Wire.setClock(400000);  // I2C 快速模式 400kHz

    bool successA = initMPU(mpuA, 0x68, "A");
    bool successB = initMPU(mpuB, 0x69, "B");

    dmpReadyA = successA;
    dmpReadyB = successB;
    initializationError = !(successA && successB);

    return !initializationError;
}

// 单个 MPU6050 的初始化流程：
// 1. initialize() — 唤醒芯片，配置时钟源和数字低通滤波器
// 2. testConnection() — 读取 WHO_AM_I 寄存器验证 I2C 通信
// 3. dmpInitialize() — 加载 DMP 固件，配置 FIFO 和输出速率
// 4. CalibrateAccel/Gyro — 6 次采样校准零偏
// 5. setDMPEnabled(true) — 启动 DMP 开始输出姿态数据
bool DualMPU6050::initMPU(MPU6050_6Axis_MotionApps20& mpu, uint8_t address, const char* label) {
    mpu.initialize();

    if (!mpu.testConnection()) {
        return false;
    }

    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        return false;
    }

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    return true;
}

// ============================================================
// 数据更新
// ============================================================

void DualMPU6050::update() {
    if (!isReady()) return;

    // 从传感器 A 的 DMP FIFO 读取姿态数据包
    if (mpuA.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpuA.dmpGetQuaternion(&q, fifoBuffer);
        mpuA.dmpGetGravity(&gravity, &q);
        mpuA.dmpGetYawPitchRoll(yprA, &q, &gravity);
    }

    // 从传感器 B 的 DMP FIFO 读取姿态数据包
    if (mpuB.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpuB.dmpGetQuaternion(&q, fifoBuffer);
        mpuB.dmpGetGravity(&gravity, &q);
        mpuB.dmpGetYawPitchRoll(yprB, &q, &gravity);
    }
}

// ============================================================
// Roll 角度计算
// ============================================================

float DualMPU6050::getAverageRoll() const {
    if (!isReady()) return 0.0f;
    return (getRollA() + getRollB()) / 2.0f;
}

float DualMPU6050::getRollA() const {
    if (!dmpReadyA) return 0.0f;
    return (yprA[2] * 180.0f / M_PI) - rollOffsetA;  // 弧度转度并减去偏移
}

float DualMPU6050::getRollB() const {
    if (!dmpReadyB) return 0.0f;
    return (yprB[2] * 180.0f / M_PI) - rollOffsetB;
}

// ============================================================
// 状态查询
// ============================================================

bool DualMPU6050::isReady() const {
    return dmpReadyA && dmpReadyB;
}

bool DualMPU6050::hasError() const {
    return initializationError;
}

bool DualMPU6050::isConnected() {
    if (!dmpReadyA || !dmpReadyB) return false;
    return mpuA.testConnection() && mpuB.testConnection();
}

// ============================================================
// 软件归零
// ============================================================

void DualMPU6050::zero() {
    if (!isReady()) return;
    rollOffsetA = yprA[2] * 180.0f / M_PI;
    rollOffsetB = yprB[2] * 180.0f / M_PI;
}
