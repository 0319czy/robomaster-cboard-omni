"""Horco v0.2 热身：只发送 DAP_Info，不操作目标板，不分离内核驱动。"""

import sys, time
import usb.core, usb.util
from hardware_config import require_probe_serial


def warmup(duration=3.0):
    serial = require_probe_serial()
    devices = usb.core.find(find_all=True, idVendor=0xFAED, idProduct=0x4870)
    device = next((d for d in devices if d.serial_number == serial), None)
    if device is None:
        raise RuntimeError("没有找到指定序列号的 Horco 下载器")
    sent = replied = 0
    usb.util.claim_interface(device, 0)
    try:
        until = time.monotonic() + duration
        packet = bytes([0, 1] + [0] * 62)  # DAP_Info / Vendor，只询问下载器信息。
        while time.monotonic() < until:
            device.write(0x01, packet, timeout=500)
            sent += 1
            try:
                response = device.read(0x81, 64, timeout=200)
                if len(response) >= 2 and response[0] == 0:
                    replied += 1
            except usb.core.USBTimeoutError:
                pass
            time.sleep(0.15)
    finally:
        usb.util.release_interface(device, 0)
        usb.util.dispose_resources(device)
    if not replied:
        raise RuntimeError(f"DAP_Info 热身无应答（发送 {sent} 条），未尝试目标操作")
    return f"DAP_Info 热身：发送 {sent}，应答 {replied}"


if __name__ == "__main__":
    try:
        print(warmup(float(sys.argv[1]) if len(sys.argv) > 1 else 3.0))
    except Exception as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
