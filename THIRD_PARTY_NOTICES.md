# 第三方组件说明

固件通过PlatformIO获取ST STM32平台、STM32CubeF4 HAL/CMSIS与ARM工具链；各组件保留其原有授权和声明，本仓库不重新授权这些组件。

本机optional_usb_cdc目录包含历史ST USB Device组件，未加入当前构建，默认不放入公共提交或上传包。其文件原有版权头保持原样；重新分发前应补齐对应组件的原始授权文件。

Python运行/格式化依赖列于requirements.txt和requirements-dev.txt，未将这些第三方包源码复制进仓库。项目自身的开源许可证尚由所有者确定。
