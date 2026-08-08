# 手眼标定工具设计说明

## 目标

在现有 Qt 6 Widgets 空工程上，构建一个可独立运行的手眼标定桌面工具。第一阶段面向已有机器人位姿和标定板位姿数据的离线分析，不绑定 ROS、相机 SDK 或特定机器人品牌；用户可以导入多组运动数据，选择多种算法，比较结果并导出最终变换。

## GitHub 调研结论

- [OpenCV calibration_handeye.cpp](https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/calibration_handeye.cpp) 是五种经典算法的权威实现来源：Tsai、Park、Horaud、Andreff、Daniilidis。
- [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) 验证了桌面工具应覆盖眼在手/眼在外语义、采样管理、结果保存和精度评估。
- [JonesCVBS/HandEyeCalibration-using-OpenCV](https://github.com/JonesCVBS/HandEyeCalibration-using-OpenCV) 体现了“图像/棋盘格检测 → 相机位姿 → 手眼求解 → 重投影误差”的数据流。
- [agilexrobotics/handeye_calibration_ros](https://github.com/agilexrobotics/handeye_calibration_ros) 体现了采样增删、批量计算和机器人场景中的操作流程。

## 第一阶段功能

1. 数据集管理：新建/清空、CSV 导入、示例数据生成、样本表格、删除选中样本。
2. 输入格式：每行包含 `gripper2base` 与 `target2cam` 的旋转向量和位移，角度使用弧度，位移使用用户选择的单位；导入时显示解析错误。
3. 算法：OpenCV 五种 `calibrateHandEye` 方法；界面支持单算法计算和全部算法批量计算。
4. 结果：显示相机到夹爪的 4×4 齐次矩阵、平移、旋转向量、算法状态和计算耗时。
5. 评估：使用 `A_i X ≈ X B_i` 的运动一致性计算旋转/平移残差，按算法给出均方根误差。
6. 导出：JSON 保存完整数据集和算法结果，CSV 导出结果摘要；复制矩阵到剪贴板。
7. UI：中文 Qt Widgets 主窗口、状态栏、菜单、参数面板、样本表、结果表、日志面板；保留 Qt Linguist 翻译入口。

## 架构与数据流

```text
MainWindow
  ├─ CalibrationSessionModel   数据集与结果的 Qt 模型
  ├─ CalibrationService        输入校验、OpenCV 算法调用、误差评估
  ├─ DatasetIo                  CSV/JSON 读写
  └─ MatrixUtils                位姿、旋转向量、齐次矩阵转换
```

用户导入或生成数据后，`CalibrationSessionModel` 保存规范化样本；点击计算时，`CalibrationService` 复制只读输入，在工作线程中调用 OpenCV，返回 `CalibrationResult`；主线程更新结果表、矩阵预览和日志。第一版为了减少状态复杂度，计算使用 `QtConcurrent`，UI 通过信号槽接收结果。

## 算法语义

程序统一采用 OpenCV 文档定义的输入方向：`R_gripper2base/t_gripper2base` 与 `R_target2cam/t_target2cam`，输出 `R_cam2gripper/t_cam2gripper`。眼在手模式直接使用该输出；眼在外模式在结果层提供逆变换和明确标签，避免把矩阵方向混淆。所有样本至少需要 3 组，推荐 10 组以上且包含绕多个轴的旋转。

## 错误处理与验证

- CSV 列数、数值、矩阵可逆性、样本数不足时阻止计算并给出行号。
- OpenCV 异常转换为用户可读的状态日志，不让异常穿透 UI 线程。
- 无 OpenCV 时 CMake 配置明确失败并提示安装方式。
- 使用确定性的合成位姿测试数据验证五种算法均能返回有效 4×4 变换；对称/退化运动给出警告。
- 构建后运行桌面程序，至少验证启动、示例数据、批量计算、导入导出和清空流程。

## 后续扩展

相机实时采集、棋盘格/圆点/ArUco 检测、机器人 TCP/串口/ROS2 接入、Robot-World/Hand-Eye 联合标定和非线性优化作为第二阶段，接口预留但不阻塞第一阶段交付。
