# 构建、烧录与版本

## 主机构建

```bash
pip install -r requirements.txt
bash tools/verify.sh
pio run -e cboard_diagnostic
```

`verify.sh`运行控制算法和记录器的C主机测试，再构建控制固件、检查向量表。无下载器时也能执行。诊断环境在最终CAN打包处强制零电流。

## 下载器配置

```bash
cp tools/hardware_config.example.json tools/local_config.json
# 将本机下载器序列号填入local_config.json
python3 tools/deploy.py --env cboard_chassis
```

也可设置环境变量 `CMSIS_DAP_SERIAL`。工具路径默认来自PlatformIO目录，可通过 `PLATFORMIO_CORE_DIR`、`OPENOCD`、`ARM_NM`覆盖。Shell验证入口允许设置 `PIO`路径。

当前部署链路针对已验证的Horco v0.2 CMSIS-DAP，包含DAP_Info热身；不宣称其它下载器均已适配。硬件工具共用访问锁，不终止其它OpenOCD。

部署顺序：固定待写镜像 → 整片1MiB连读三次、丢弃首读并要求后两次一致 → 写入 → 同样双读一致且逐字节匹配 → 复位运行。备份或校验失败时不跳过步骤。完整备份保留在本机backups目录。

## 当前两个版本

| 内容 | SHA256 |
|---|---|
| 板上已实测30ms补偿版 | `5132a277eb1456089a3a09705c2683ff95cffa9623d9cf66858801c7b67fdb7f` |
| 整理后源码构建 | `69198bb5d62075a6e987dc8000ee94d66c26d96b8114cf84d8d0c1439d4bc147` |

整理未改控制参数；历史诊断寄存器读取改用HAL宏后，二进制发生变化。用户结束实车调试后，没有再次烧录整理版。两版ELF地址不可混用，采集工具会校验Flash与配套BIN；需分析旧板上RAM时，使用本机归档的旧版ELF。

## 代码格式与GitHub

```bash
pip install -r requirements-dev.txt
clang-format --dry-run --Werror src/*.c include/*.h tests/*.c
black --check tools
python3 -m compileall -q tools
```

本机序列号、构建产物、固件备份和未筛选原始记录默认忽略。上传前从Git文件清单核对内容；不要把本机整个底盘目录直接打包提交。项目级开源许可证尚待所有者确定。

生成按Git清单筛选的上传压缩包：

```bash
python3 tools/package_project.py --out exports/github-source.zip
```

压缩包排除本机硬件身份、整片备份、构建缓存和历史USB组件；原件仍留在工作目录。
