"""按Git清单生成源码上传包，排除本机硬件配置、备份与构建产物。"""

import argparse
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRIVATE_ROOTS = {".git", ".pio", ".venv", "backups", "local_history", "exports", "optional_usb_cdc"}
ARTIFACT_SUFFIXES = {".bin", ".elf", ".hex", ".o", ".a", ".pyc"}


def source_files():
    """兼容已提交和未提交的新仓库；额外检查不依赖Git忽略规则的私有边界。"""
    output = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=ROOT
    )
    result = []
    for name in sorted(set(output.decode().split("\0")) - {""}):
        path = Path(name)
        if path.parts[0] in PRIVATE_ROOTS or "__pycache__" in path.parts:
            continue
        if name == "tools/local_config.json" or path.suffix in ARTIFACT_SUFFIXES:
            continue
        if path.parts[0] == "logs" and name not in {
            "logs/README.md",
            "logs/INDEX.md",
            "logs/latest_configuration.json",
        }:
            continue
        absolute = ROOT / path
        if absolute.is_symlink():
            raise RuntimeError(f"上传包不收集符号链接：{name}")
        if absolute.is_file():
            result.append(path)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "exports/github-source.zip")
    args = parser.parse_args()
    output = args.out.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    files = [path for path in source_files() if (ROOT / path).resolve() != output]
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in files:
            archive.write(ROOT / path, str(Path("robomaster-cboard-omni") / path))
    with zipfile.ZipFile(output) as archive:
        if archive.testzip() is not None:
            raise RuntimeError("上传包完整性检查失败")
    print(f"上传包：{output}\n文件：{len(files)}，大小：{output.stat().st_size} 字节")


if __name__ == "__main__":
    main()
