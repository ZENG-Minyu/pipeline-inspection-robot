"""
管道检测机器人 — 步数→柱坐标转换工具
======================================
读取 serial_logger.py 保存的日志文件，将四轴电机步数转换为
管道内壁柱坐标系 (r, θ, z) 下的时间-空间坐标。

坐标系定义：
  r  (径向) = 管道内半径，固定常值（PIPE_RADIUS = 234.0 mm）
  θ  (角度) = Y 电机步数换算（STEPS_PER_Y_REVOLUTION = 2230 steps/360°）
  z  (轴向) = A 电机步数换算（沿管道行进距离）

步数→物理量换算公式：
  z_mm  = steps_A × π × WHEEL_RADIUS / 800
  θ_deg = steps_Y × 360.0 / STEPS_PER_Y_REVOLUTION
  θ_rad = steps_Y × 2π / STEPS_PER_Y_REVOLUTION

推导过程：
  - A 电机使用 1/8 微步：200 全步/转 × 8 = 1600 微步/转
  - 每步对应轮子行进 = (2π × 35mm) / 1600 = π × 35 / 800 mm
  - Y 电机减速比通过实际测试标定获得（包含齿轮啮合间隙等因素）

输出格式：CSV（逗号分隔值），可直接导入 Excel/Python/MATLAB 分析

用法：
  1. 修改下方 INPUT_FILE 为实际日志文件名
  2. .venv/bin/python log_to_coords.py

作者：ZENG Minyu
"""

import re
import math
from datetime import datetime

# ===================== 用户可调参数 =====================
INPUT_FILE = "log_data_2026-04-28T13-38-05.txt"   # 输入的日志文件名（修改为实际文件）
OUTPUT_FILE = "coords_output.csv"                  # 输出的坐标 CSV 文件名

WHEEL_RADIUS = 35.0         # [mm] 驱动轮半径（轮径 70mm 的一半）
PIPE_RADIUS = 234.0         # [mm] 管道内半径（内径 468mm），即柱坐标中的固定 r 值
STEPS_PER_Y_REVOLUTION = 2230  # Y 电机（旋转盘）每 360° 所需步数（实测标定值）
# ======================================================

# 派生常量
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS  # 轮子周长 [mm]
STEPS_PER_MOTOR_REVOLUTION = 200 * 8   # A 电机每转步数：200 全步 × 1/8 微步 = 1600


def steps_to_z(steps_a: float) -> float:
    """
    A 电机步数 → 轴向位移 z [mm]

    推导：
      每转 = 1600 步（1/8 微步）
      每转行进 = 2π × WHEEL_RADIUS = π × 70 mm
      每步行进 = (π × 70) / 1600 = π × 35 / 800 mm
      z = steps_a × π × WHEEL_RADIUS / 800

    参数：
      steps_a: A 电机的累计步数（可为负，表示后退）

    返回：
      轴向位移 [mm]（正值 = 前进，负值 = 后退）
    """
    return steps_a * math.pi * WHEEL_RADIUS / 800.0


def steps_to_theta_deg(steps_y: float) -> float:
    """
    Y 电机步数 → 旋转角度 [度]

    参数：
      steps_y: Y 电机的累计步数（正 = CCW，负 = CW）

    返回：
      旋转角度 [度]，0° 对应初始朝向
    """
    return steps_y * 360.0 / STEPS_PER_Y_REVOLUTION


def steps_to_theta_rad(steps_y: float) -> float:
    """
    Y 电机步数 → 旋转角度 [弧度]

    参数：
      steps_y: Y 电机的累计步数

    返回：
      旋转角度 [弧度]
    """
    return steps_y * 2.0 * math.pi / STEPS_PER_Y_REVOLUTION


def parse_log_line(line: str):
    """
    解析单行日志，提取时间戳和四轴步数。

    支持的格式：
      [+0.000] X: 123, Y: 456, Z: 0, A: 0

    参数：
      line: 一行日志文本

    返回：
      (elapsed_sec, dict_of_steps) 或 None（解析失败）
      dict_of_steps 包含键 'X', 'Y', 'Z', 'A'，值为 int
    """
    m = re.match(
        r'^\[([+-]?\d+\.\d+)\]\s+'
        r'X:\s*(-?\d+)'
        r'\s*,\s*Y:\s*(-?\d+)'
        r'\s*,\s*Z:\s*(-?\d+)'
        r'\s*,\s*A:\s*(-?\d+)',
        line
    )
    if not m:
        return None

    elapsed = float(m.group(1))
    steps = {
        'X': int(m.group(2)),
        'Y': int(m.group(3)),
        'Z': int(m.group(4)),
        'A': int(m.group(5)),
    }
    return elapsed, steps


def main():
    # 读取日志文件
    with open(INPUT_FILE) as f:
        lines = f.readlines()

    print(f"Read {len(lines)} lines from {INPUT_FILE}")

    # 解析每一行
    parsed = []
    for line in lines:
        result = parse_log_line(line.strip())
        if result:
            parsed.append(result)

    print(f"Parsed {len(parsed)} motor-state lines")

    if not parsed:
        print("No valid data found — check log format.")
        return

    # 转换为柱坐标并写入 CSV
    with open(OUTPUT_FILE, 'w') as f:
        # CSV 表头
        f.write("elapsed_s,r_mm,theta_deg,theta_rad,z_mm,"
                "x_steps,y_steps,z_steps,a_steps\n")

        for elapsed, steps in parsed:
            r = PIPE_RADIUS                           # 径向 = 管道半径（固定）
            theta_deg = steps_to_theta_deg(steps['Y'])  # 角度 [度]
            theta_rad = steps_to_theta_rad(steps['Y'])  # 角度 [弧度]
            z = steps_to_z(steps['A'])                   # 轴向位移 [mm]

            f.write(
                f"{elapsed:.3f},{r:.4f},{theta_deg:.4f},{theta_rad:.6f},{z:.4f},"
                f"{steps['X']},{steps['Y']},{steps['Z']},{steps['A']}\n"
            )

    print(f"\nOutput: {OUTPUT_FILE}")
    print(f"Params used: WHEEL_RADIUS={WHEEL_RADIUS} mm, "
          f"PIPE_RADIUS={PIPE_RADIUS} mm, "
          f"STEPS_PER_Y_REVOLUTION={STEPS_PER_Y_REVOLUTION}")


if __name__ == "__main__":
    main()
