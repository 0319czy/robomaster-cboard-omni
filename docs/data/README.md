# 筛选数据

`recordings_20261119.zip`包含两段100Hz原始JSON及对应辨识结果，已去除个人绝对路径。压缩包内数据仍保留固件哈希、单位说明和所有采样字段。

- baseline_ccw.json：未补偿，逆时针。
- prediction_cw.json：30ms预测，顺时针。
- 对应phase.json：固定杆量样本选择和等效模型结果。

解压后可用tools/analyze_mixed_drive.py与tools/identify_translation_lag.py复现。两段方向不同，不是严格同条件A/B。编码器估计不等同真实地面定位。

图表：
- [未补偿逆时针总览](../images/baseline-ccw.png)
- [预测后顺时针总览](../images/prediction-cw.png)
- [预测后陀螺仪](../images/prediction-gyro.png)
