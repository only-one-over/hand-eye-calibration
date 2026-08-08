# Qt6 手眼标定工具

这是一个基于 Qt 6 Widgets + OpenCV 的离线手眼标定桌面工具。当前版本通过 CSV/JSON 导入机器人末端与标定板位姿，支持五种 OpenCV 手眼标定算法并提供结果误差摘要。

## 已实现功能

- Tsai-Lenz、Park-Martin、Horaud、Andreff、Daniilidis 五种算法
- 眼在手/眼在外模式选择、样本表格、示例数据生成
- CSV/JSON 导入与导出
- 旋转/平移误差摘要、4×4 齐次矩阵预览、运行日志
- `--smoke-test` 无界面运行入口，用于验证五种算法和运行时依赖

## 构建

本机 Qt 6.9.3 和 OpenCV 4.12 的构建命令示例：

```powershell
cmake -S . -B build/handeye_mvp `
  -DOpenCV_DIR=C:/opencv/build/x64/vc16/lib `
  -DCMAKE_PREFIX_PATH=C:/Qt/6.9.3/msvc2022_64 `
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build/handeye_mvp --config Debug
```

如果 Qt Creator 仍缓存了错误的 `OpenCV_DIR=C:/opencv/build`，CMake 会自动回退到本机兼容目录 `C:/opencv/build/x64/vc16/lib`。也可以显式指定：

```powershell
cmake -S . -B build/handeye_mvp `
  -DHAND_EYE_OPENCV_DIR=C:/opencv/build/x64/vc16/lib
```

运行时需要将 Qt 的 `bin` 和 OpenCV 的 `build/x64/vc16/bin` 加入 `PATH`。在 Windows 上也可以把这些 DLL 复制到可执行文件目录，便于发布。

## 数据方向与 CSV 格式

每组样本包含：

```text
gripper2base = [R_gripper2base, t_gripper2base]
target2cam   = [R_target2cam, t_target2cam]
```

旋转使用 Rodrigues 旋转向量（弧度），平移使用界面中声明的单位。CSV 表头为：

```text
id,label,gripper_rx,gripper_ry,gripper_rz,gripper_tx,gripper_ty,gripper_tz,target_rx,target_ry,target_rz,target_tx,target_ty,target_tz
```

OpenCV 输出的核心结果方向为 `camera → gripper`。建议采集至少 10 组、绕多个轴有明显旋转变化的样本；少于 3 组或运动退化时程序会阻止计算或给出警告。

## 调研与设计

设计说明和分步实施计划位于 `docs/plans/`。GitHub 调研重点参考了 OpenCV 的手眼实现、easy_handeye2、JonesCVBS 的 OpenCV 示例和 AgileX ROS2 工具。
