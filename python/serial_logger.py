"""
管道检测机器人 — 串口数据采集工具
==================================
从 Arduino Mega 2560 串口持续读取电机状态信息，每行附带相对时间戳（从首行开始计时）。
按 Ctrl+C 停止记录后，自动保存为带时间戳的日志文件。

依赖：pyserial（通过 `pip install -r requirements.txt` 安装）

用法：
  # 自动检测 Arduino 串口
  .venv/bin/python serial_logger.py

  # 手动指定串口路径
  .venv/bin/python serial_logger.py --port /dev/cu.usbmodem11201

  # 指定波特率（默认 115200）
  .venv/bin/python serial_logger.py --baud 115200

输出格式示例：
  [+0.000] X: 0, Y: 0, Z: 0, A: 0
  [+1.234] X: 0, Y: 30, Z: 0, A: 100
  [+2.567] X: 0, Y: 60, Z: 0, A: 100

输出文件：log_data_YYYY-MM-DDTHH-MM-SS.txt（保存在脚本同目录下）

作者：ZENG Minyu
"""

import serial
import serial.tools.list_ports
import argparse
import time
import signal
from datetime import datetime


def find_arduino_port():
    """
    自动检测 Arduino 串口。

    扫描所有可用串口，优先匹配：
    1. 设备路径中包含 'usb' 的端口
    2. 设备描述中包含 'Arduino' 的端口

    返回：匹配到的端口路径（如 /dev/cu.usbmodem11201），未找到则返回 None
    """
    ports = serial.tools.list_ports.comports()
    print("Available ports:")
    for p in ports:
        print(f"  {p.device}  -  {p.description}")

    for p in ports:
        desc = p.description.lower()
        device = p.device.lower()
        if 'usb' in device or 'arduino' in desc:
            print(f"\nAuto-selected: {p.device}")
            return p.device

    return None


def main():
    parser = argparse.ArgumentParser(description="FYDP Serial Logger — 管道机器人串口数据采集")
    parser.add_argument("--port", "-p", help="串口路径（如 /dev/cu.usbmodem11201），不指定则自动检测")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="波特率（默认 115200）")
    args = parser.parse_args()

    BAUD = args.baud

    # ----- 检测串口 -----
    PORT = args.port or find_arduino_port()

    if PORT is None:
        print("\nNo Arduino detected. Specify port with --port")
        return

    print(f"Using {PORT} @ {BAUD} baud")

    # ----- 打开串口 -----
    # timeout=0.05：读取超时 50ms，确保 readline() 不会无限阻塞
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
        time.sleep(2)  # 等待 Arduino 复位完成
        ser.reset_input_buffer()  # 清空复位期间产生的垃圾数据
        print("Serial connected. Recording... (press Ctrl+C to stop)\n")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    # ----- 读取循环 -----
    records = []      # 存储 (elapsed_seconds, line_string) 元组
    stop_flag = False
    start_time = None  # 首行到达的时间戳（time.monotonic()）

    def handle_sigint(sig, frame):
        """Ctrl+C 信号处理：设置停止标志，优雅退出"""
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGINT, handle_sigint)

    line_count = 0
    while not stop_flag:
        try:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    now = time.monotonic()
                    if start_time is None:
                        start_time = now  # 以第一条有效数据的行为时间零点
                    elapsed = now - start_time
                    records.append((elapsed, line))
                    line_count += 1
                    # 每 50 行输出进度提示（避免刷屏）
                    if line_count % 50 == 0:
                        print(f"\rReading... {line_count} lines", end="", flush=True)
        except serial.SerialException:
            break  # 串口断开，停止读取

    ser.close()
    print()

    # ----- 保存到文件 -----
    ts_str = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
    log_path = f"log_data_{ts_str}.txt"

    with open(log_path, "w") as f:
        for elapsed, line in records:
            f.write(f"[+{elapsed:.3f}] {line}\n")

    print(f"\nDone. Saved {line_count} lines.")
    print(f"Log: {log_path}")


if __name__ == "__main__":
    main()
