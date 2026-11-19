"""下载器共用配置。仅使用指定 CMSIS-DAP，禁止同时占用另一个 OpenOCD。"""

from pathlib import Path
import contextlib, fcntl, subprocess
from hardware_config import PROJECT_ROOT as ROOT, OPENOCD, SERIAL, require_probe_serial

FLASH_SIZE = 0x100000
SNAPSHOT_SIZE = 1088


def tcl_path(path):
    text = str(Path(path).resolve())
    if any(c in text for c in "{}\n\r"):
        raise ValueError("路径中不允许包含 Tcl 分隔字符")
    return "{" + text + "}"


@contextlib.contextmanager
def probe_lock():
    # 所有本工程工具共用同一锁。外部智能体也应遵守；此外检查现存 OpenOCD。
    with open(ROOT.parent / ".cmsis-dap.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("下载器锁已被占用，请等待当前操作完成")
        result = subprocess.run(["pgrep", "-x", "openocd"], capture_output=True, text=True)
        if result.returncode == 0:
            raise RuntimeError("检测到其他 OpenOCD，未连接或终止它：" + result.stdout.strip())
        yield


def openocd(commands, log, timeout=180):
    require_probe_serial()
    from dap_warmup import warmup

    warmup_note = warmup()
    args = [str(OPENOCD)]
    setup = [
        "gdb port disabled",
        "tcl port disabled",
        "telnet port disabled",
        "adapter driver cmsis-dap",
        "adapter serial " + SERIAL,
        "transport select swd",
    ]
    for command in setup:
        args += ["-c", command]
    args += ["-f", "target/stm32f4x.cfg", "-c", "adapter speed 1000"]
    for command in commands:
        args += ["-c", command]
    result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    text = warmup_note + "\n" + result.stdout + result.stderr
    Path(log).parent.mkdir(parents=True, exist_ok=True)
    Path(log).write_text(text)
    if result.returncode:
        raise RuntimeError("OpenOCD 失败，日志：" + str(log) + "\n" + text[-3000:])
    return text
