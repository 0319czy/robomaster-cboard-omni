# RoboMaster C 板四全向轮底盘

基于 STM32F407、STM32 HAL 和 PlatformIO 的四轮底盘工程。支持前后/左右平移、双向自旋，以及自旋时保持参考方向移动。

右摇杆转向会改变平移参考航向；SB 自旋保持该参考航向。当前已加入根据陀螺角速度计算的 30 ms 平移预测补偿，以及板内 100 Hz 同步记录。

## 硬件

| 项目 | 当前配置 |
|---|---|
| 主控 | RoboMaster C 型板，STM32F407IGH6 |
| 时钟 | 12 MHz HSE，168 MHz 系统时钟 |
| 电机/电调 | 4 × M3508 / C620 |
| CAN | CAN1，PD0/PD1，1 Mbps |
| 电机位置 | ID1 左后、ID2 左前、ID3 右前、ID4 右后 |
| 轮组 | 四全向轮，外径 150 mm |
| 遥控 | USART3 PC11，反相 SBUS，100 kbit/s、8E2 |
| IMU | 板载 BMI088，SPI1 |

半长 150 mm、半宽 160 mm 为当前近似几何参数，尚未做精密标定。

## 快速构建

主机工具按 Python 3.10 配置依赖；现场曾使用 Linux/Python 3.12 的系统包运行采集工具。

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
bash tools/verify.sh
```

也可直接执行 `pio run -e cboard_chassis`。本项目使用 STM32Cube HAL 框架，当前没有 CubeMX `.ioc` 文件。

烧录前先按 [构建与烧录](docs/build-and-flash.md) 配置自己的下载器序列号。本机配置不提交到 GitHub。

## 目录

```text
src/       HAL外设接入、遥控映射、混控、速度环和记录器
include/   模块接口、单位约定和集中参数
tools/     烧录、只读采集、离线分析和绘图
tests/     控制逻辑与记录器的主机回归测试
docs/      架构、操作、数据格式与开发日志
logs/      本机原始记录；公共仓库仅保留索引和配置快照
backups/   本机固件/源码备份，默认不提交
web/       历史串口控制页面原型，非当前主要操作入口
```

## 文档入口

- [文档目录](docs/README.md)
- [模块架构与代码规范](docs/architecture.md)
- [遥控操作与参数](docs/operation.md)
- [构建、烧录和版本说明](docs/build-and-flash.md)
- [100 Hz 同步采集与离线分析](docs/telemetry.md)
- [开发日志](docs/journal/README.md)
- [当前交接](HANDOFF.md)
- [已知限制](docs/known-limitations.md)

## 实测状态

100 Hz 记录已验证为 1000 帧/9.99 秒，控制周期采样值为 2 ms。未补偿逆时针样本的平移通道等效滞后约 35 ms；加入 30 ms 预测后，顺时针样本的编码器估计方向误差约 −3.8°。两段旋向不同，不能当作严格同条件 A/B 试验；真实地面轨迹仍未通过外部定位量化。

用户已表示当前可以使用并结束调试。最后一段已保存记录末态为四轮目标、转速和发送电流全零，fault=0。整理版源码构建与板上已验证固件的版本区分见 [版本说明](docs/build-and-flash.md)。

本仓库尚未指定项目级开源许可证；第三方依赖和历史组件说明见 [第三方说明](THIRD_PARTY_NOTICES.md)。
