"""为离线波形图选择中文字体；缺少字体时保留Matplotlib默认回退。"""

import os
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib import font_manager


def configure_plot_font():
    """优先使用CHASSIS_FONT指定的字体，其次查找常见系统中文字体。"""
    candidates = [
        os.environ.get("CHASSIS_FONT", ""),
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "C:/Windows/Fonts/msyh.ttc",
        "/System/Library/Fonts/PingFang.ttc",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            font_manager.fontManager.addfont(candidate)
            name = font_manager.FontProperties(fname=candidate).get_name()
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    else:
        plt.rcParams["font.sans-serif"] = [
            "Noto Sans CJK SC",
            "Microsoft YaHei",
            "PingFang SC",
            "DejaVu Sans",
        ]
    plt.rcParams["axes.unicode_minus"] = False
