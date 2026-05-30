/*
  步进电机驱动模块
  ================
  封装 A4988 步进电机驱动器的底层控制逻辑，是整个软件系统的基石类。

  硬件接口：每个电机占用两个数字引脚（STEP 脉冲 + DIR 方向），
           通过 CNC Shield v3 连接至 A4988 驱动器。

  支持的运行模式：
  - 绝对位置模式 (moveTo)：移动到指定步数位置（开环）
  - 连续运转模式 (run)：持续正/反转，直至手动停止（用于 PS2 摇杆控制）
  - 阻塞式步进 (stepBlocking)：精确走完指定步数后返回（用于自动化序列和点动）

  中断驱动架构：
  - update()：主循环调用（当前已弃用，保留用于无中断场景的兼容）
  - updateDirectInInterrupt()：由 Timer1 100μs 定时器中断调用，直接生成脉冲，
    确保脉冲时序精度不受主循环频率波动影响

  作者：ZENG Minyu
*/
#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>

// 步进电机转向枚举
// 值与 digitalWrite 的 HIGH/LOW 直接对应，简化方向引脚的控制
enum Direction {
    CW = HIGH,   // 顺时针 / 正转
    CCW = LOW    // 逆时针 / 反转
};

class StepperMotor {
public:
    // 构造函数：绑定 STEP 和 DIR 引脚
    // stepPin — A4988 的 STEP 引脚，每个上升沿驱动电机前进一个微步
    // dirPin  — A4988 的 DIR 引脚，HIGH = CW, LOW = CCW
    StepperMotor(int stepPin, int dirPin);

    // 初始化：将 STEP 和 DIR 引脚设置为 OUTPUT 模式
    void begin();

    // 绝对位置模式：命令电机移动到目标步数位置（以当前步数为原点）
    // targetPosition — 目标步数（正/负表示相对于原点的位置）
    // intervalUs — 脉冲间隔（微秒），越小速度越快
    void moveTo(long targetPosition, unsigned long intervalUs);

    // 连续运转模式：电机持续转动直到调用 stop()
    // dir — 转向（CW 或 CCW）
    // intervalUs — 脉冲间隔（微秒）
    void run(Direction dir, unsigned long intervalUs);

    // 停止电机（将 _running 标志置为 false）
    void stop();

    // 主循环更新函数（非中断模式，当前已弃用）
    // 每次调用最多产生一个脉冲，脉冲时序精度取决于主循环频率
    void update();

    // 定时器中断更新函数（当前主要使用的脉冲生成方式）
    // 由 Timer1 每 100μs 调用一次，直接在 ISR 中生成完整脉冲
    // 设计要点：每次 ISR 执行时间在纳秒级，不影响主循环的 PS2 通信和 OLED 刷新
    void updateDirectInInterrupt();

    // 查询电机是否正在运转（连续模式或位置模式中途）
    bool isRunning() const;

    // 查询电机是否处于连续运转模式（run() 启动的模式）
    bool isContinuousMode() const;

    // 获取当前累计步数（可为负值）
    long getCurrentPosition() const;

    // 将当前步数计数器归零（通过 cli/sei 保证中断安全）
    void resetPosition();

    // 阻塞式步进：在当前线程中精确走完指定步数，期间禁用中断以避免与 ISR 冲突
    // steps — 步数（正 = CW，负 = CCW）
    // intervalUs — 脉冲间隔（微秒），值越大速度越慢，需考虑电机的启动扭矩
    // shouldContinue — 可选回调函数，返回 false 时中途终止步进（当前未使用，传入 nullptr）
    void stepBlocking(long steps, unsigned long intervalUs, bool (*shouldContinue)() = nullptr);

    // 设置步进脉冲宽度（微秒），默认 200μs
    // A4988 要求 STEP 引脚高电平至少维持约 1μs 才能可靠检测，
    // 200μs 提供了足够的安全裕度
    void setPulseWidth(unsigned long pulseWidthUs);

private:
    // 生成一个完整的步进脉冲（STEP 引脚 HIGH → 延时 → LOW）
    void stepOnce();

    int _stepPin;                // STEP 引脚编号
    int _dirPin;                 // DIR 引脚编号

    long _currentPosition;       // 当前累计位置（步数，可为负）
    long _targetPosition;        // 绝对位置模式的目标步数

    unsigned long _intervalUs;   // 步进间隔（脉冲周期，微秒）
    unsigned long _lastStepTime; // 上一次产生脉冲的时间戳（micros()）
    unsigned long _pulseWidthUs; // 脉冲高电平持续时间（微秒），默认 200

    bool _running;               // 电机是否在运转
    bool _continuousMode;        // 是否为连续运转模式（true = 不会自动停止）
    Direction _dir;              // 当前转向
};

#endif
