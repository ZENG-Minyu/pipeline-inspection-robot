#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// 创建两个 MPU 对象，分别对应两个 I2C 地址
MPU6050 mpuA(0x68);
MPU6050 mpuB(0x69);

// 状态变量
bool dmpReadyA = false;
bool dmpReadyB = false;
uint8_t fifoBuffer[64]; // 可以共用一个缓冲区来节省内存

// 姿态变量
Quaternion q;           
VectorFloat gravity;    
float yprA[3];           
float yprB[3];           

void setup() {
    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);

    Serial.println(F("Initializing MPU6050 A (0x68)..."));
    mpuA.initialize();
    if (mpuA.testConnection() && mpuA.dmpInitialize() == 0) {
        mpuA.CalibrateAccel(6);
        mpuA.CalibrateGyro(6);
        mpuA.setDMPEnabled(true);
        dmpReadyA = true;
    } else {
        Serial.println(F("MPU6050 A failed!"));
    }

    Serial.println(F("Initializing MPU6050 B (0x69)..."));
    mpuB.initialize();
    if (mpuB.testConnection() && mpuB.dmpInitialize() == 0) {
        mpuB.CalibrateAccel(6);
        mpuB.CalibrateGyro(6);
        mpuB.setDMPEnabled(true);
        dmpReadyB = true;
    } else {
        Serial.println(F("MPU6050 B failed!"));
    }

    if (dmpReadyA && dmpReadyB) Serial.println(F("Both DMPs Ready!"));
}

void loop() {
    if (!dmpReadyA || !dmpReadyB) return;

    bool newData = false;

    // 读取 A 组数据
    if (mpuA.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpuA.dmpGetQuaternion(&q, fifoBuffer);
        mpuA.dmpGetGravity(&gravity, &q);
        mpuA.dmpGetYawPitchRoll(yprA, &q, &gravity);
        newData = true;
    }

    // 读取 B 组数据
    if (mpuB.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpuB.dmpGetQuaternion(&q, fifoBuffer);
        mpuB.dmpGetGravity(&gravity, &q);
        mpuB.dmpGetYawPitchRoll(yprB, &q, &gravity);
        newData = true;
    }

    // 只有当有新数据时才打印，并符合你的 Plotter 格式
    if (newData) {
        Serial.print(F(">YawA:"));   Serial.print(yprA[0] * 180/M_PI);
        Serial.print(F(",PitchA:")); Serial.print(yprA[1] * 180/M_PI);
        Serial.print(F(",RollA:"));  Serial.print(yprA[2] * 180/M_PI);
        
        Serial.print(F(",YawB:"));   Serial.print(yprB[0] * 180/M_PI);
        Serial.print(F(",PitchB:")); Serial.print(yprB[1] * 180/M_PI);
        Serial.print(F(",RollB:"));  Serial.println(yprB[2] * 180/M_PI);
    }
}