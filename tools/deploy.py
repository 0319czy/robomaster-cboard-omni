"""备份、写入并逐字节校验底盘固件。控制版本会在遥控全回中后解锁，诊断版本强制零电流。"""

import sys, datetime, hashlib, argparse
from common import ROOT, probe_lock, openocd, tcl_path


def stable_read(label, size, stamp):
    paths = [ROOT / "backups" / f"{label}_{stamp}_{i}.bin" for i in (1, 2, 3)]
    # 同一次连接连读三次：丢弃已知可能不稳定的首读，后两次必须一致。
    openocd(
        ["init", "reset halt"]
        + [f"dump_image {tcl_path(p)} 0x08000000 {size}" for p in paths]
        + ["shutdown"],
        ROOT / "logs" / f"{label}_{stamp}.log",
    )
    a, b = paths[1].read_bytes(), paths[2].read_bytes()
    if len(a) != size or a != b:
        raise RuntimeError("后两次读回不一致；MCU 保持停机，未恢复运行")
    paths[2].with_suffix(".sha256").write_text(
        hashlib.sha256(b).hexdigest() + "  " + paths[2].name + "\n"
    )
    return paths[2], b


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--restore-listener", action="store_true")
    p.add_argument(
        "--env", choices=["cboard_chassis", "cboard_diagnostic"], default="cboard_chassis"
    )
    args = p.parse_args()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    source = (
        ROOT.parent / "can_status_monitor/.pio/build/cboard_can_rx/firmware.bin"
        if args.restore_listener
        else ROOT / f".pio/build/{args.env}/firmware.bin"
    )
    data = source.read_bytes()
    if not 8 < len(data) < 0x40000:
        raise RuntimeError("镜像大小越界（上限 256 KiB）")
    sp, ep = int.from_bytes(data[:4], "little"), int.from_bytes(data[4:8], "little")
    if not (0x20000000 < sp <= 0x20020000 and 0x08000000 < ep < 0x08040000 and ep & 1):
        raise RuntimeError("向量无效")
    ROOT.joinpath("backups").mkdir(exist_ok=True)
    ROOT.joinpath("logs").mkdir(exist_ok=True)
    # 固定本次待写镜像，避免另一个智能体重新构建时改变烧写内容。
    image = ROOT / "backups" / f"deploy_image_{stamp}.bin"
    image.write_bytes(data)
    with probe_lock():
        backup, _ = stable_read("before_deploy", 0x100000, stamp)
        print("完整 1 MiB 备份（双读一致）：", backup, flush=True)
        openocd(
            [
                "init",
                "reset halt",
                f"flash write_image erase {tcl_path(image)} 0x08000000 bin",
                "shutdown",
            ],
            ROOT / "logs" / f"deploy_write_{stamp}.log",
        )
        readback, actual = stable_read("deploy_readback", len(data), stamp)
        if actual != data:
            raise RuntimeError(f"逐字节校验失败，MCU 保持停机；读回文件 {readback}")
        print("镜像逐字节校验通过 SHA256:", hashlib.sha256(data).hexdigest(), flush=True)
        openocd(["init", "reset run", "shutdown"], ROOT / "logs" / f"deploy_run_{stamp}.log")
    print(
        "纯监听已运行"
        if args.restore_listener
        else (
            "零电流诊断版已运行"
            if args.env == "cboard_diagnostic"
            else "控制版已运行：全回中 500 ms 后解锁"
        )
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
