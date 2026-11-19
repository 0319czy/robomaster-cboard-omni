"""本机硬件与工具路径配置；设备身份不写入公共仓库。"""

import json
import os
import re
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOCAL_CONFIG = PROJECT_ROOT / "tools" / "local_config.json"
local_config = json.loads(LOCAL_CONFIG.read_text()) if LOCAL_CONFIG.exists() else {}
CORE_DIR = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
OPENOCD = Path(os.environ.get("OPENOCD", CORE_DIR / "packages/tool-openocd/bin/openocd"))
ARM_NM = Path(
    os.environ.get("ARM_NM", CORE_DIR / "packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm")
)
SERIAL = os.environ.get("CMSIS_DAP_SERIAL", local_config.get("cmsis_dap_serial", ""))


def require_probe_serial():
    """明确选择下载器；避免多设备环境下烧到未指定的板卡。"""
    if not SERIAL or not re.fullmatch(r"[A-Za-z0-9_.:-]+", SERIAL):
        raise RuntimeError(
            "请设置 CMSIS_DAP_SERIAL，或复制 tools/hardware_config.example.json "
            "为 tools/local_config.json 后填入下载器序列号"
        )
    return SERIAL
