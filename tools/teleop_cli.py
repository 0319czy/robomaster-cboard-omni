#!/usr/bin/env python3
"""串口遥控终端版（网页不可用时的兜底）：按住 W/A/S/D/Q/E，抬起立即停。

用法: python3 tools/teleop_cli.py [/dev/ttyUSB0] [--trans 300] [--spin 800]
退出: Ctrl+C（会先发 X 停止）
"""

import argparse, os, select, sys, termios, time, tty

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial: sudo apt install python3-serial")

KEYS = {"w": "前进", "s": "后退", "a": "左平移", "d": "右平移", "q": "左旋", "e": "右旋"}
TIMEOUT_MS = 150


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--trans", type=int, default=300, help="平移速度 mm/s")
    ap.add_argument("--spin", type=int, default=800, help="自旋 mrad/s")
    ap.add_argument("--rate", type=float, default=25.0, help="发送频率 Hz")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    pressed = {}
    last_sent = 0.0
    print(
        f"已连接 {args.port} | 平移 {args.trans} mm/s 自旋 {args.spin} mrad/s | 按住 W A S D Q E，Ctrl+C 退出"
    )
    try:
        tty.setraw(fd)
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r:
                ch = os.read(fd, 1).decode(errors="ignore").lower()
                if ch == "\x03":
                    break
                if ch in KEYS:
                    pressed[ch] = not pressed.get(ch, False)
                    print(
                        "  "
                        + " ".join(
                            f'{k.upper()}:{"ON" if v else "off"}' for k, v in pressed.items() if v
                        )
                        or "  (无按键)"
                    )
                elif ch in ("\x1b",):
                    pressed.clear()
            now = time.time()
            if now - last_sent >= 1.0 / args.rate:
                last_sent = now
                vx = (args.trans if pressed.get("w") else 0) - (
                    args.trans if pressed.get("s") else 0
                )
                vy = (args.trans if pressed.get("a") else 0) - (
                    args.trans if pressed.get("d") else 0
                )
                om = (args.spin if pressed.get("q") else 0) - (args.spin if pressed.get("e") else 0)
                ser.write(b"X\n" if not (vx or vy or om) else f"V,{vx},{vy},{om}\n".encode())
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"X\n")
        ser.flush()
        ser.close()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n已发送停止，退出")


if __name__ == "__main__":
    main()
