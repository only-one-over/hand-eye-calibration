# Qt6 手眼标定工具

基于 Qt 6、OpenCV 和 Eigen 的手眼标定桌面工具，面向“输入标准化 → 标定可靠性 → 数据验证 → 工程导出”的使用流程。

## 已支持功能

- 五种 OpenCV 手眼标定算法：Tsai-Lenz、Park-Martin、Horaud、Andreff、Daniilidis。
- 自动计算全部算法，并按可靠性报告推荐结果。
- 输入姿态格式：Rodrigues、Euler XYZ、RPY、Quaternion（WXYZ）。
- 角度单位：degree/rad；长度单位：mm/m，导入时统一转换为内部的 rad/m。
- 明确记录并展示方向：`gripper → base`、`target → camera`、输出 `camera → gripper`。
- Eye-To-Hand 当前在界面中禁用，避免使用尚未完成的流程。
- 相对运动退化检测：样本数量、重复样本、旋转激励、旋转轴分布和有限值检查。
- 真实 RMSE、平均误差、最大误差、单样本残差和异常样本标记。
- 支持独立验证数据集，并输出是否通过验证。
- 合成真值测试，验证矩阵方向、数值误差、单位归一化、异常检测和退化检测。
- 机器人 Pose Adapter：Generic、Universal Robots、KUKA、FANUC。
- 矩阵导出：JSON、YAML、TXT、C++、Python；JSON 同时保存机器人、相机、单位、算法、日期和误差信息。

## 构建

项目使用 Qt 6.9.3 MSVC 2022 64-bit 和 OpenCV 4.x。CMake 会优先查找 `OpenCV_DIR`；如果发现配置文件存在但 `OpenCV_FOUND=FALSE`，会自动回退到常见的：

```text
C:/opencv/build/x64/vc16/lib
```

也可以显式指定：

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DOpenCV_DIR=C:/opencv/build/x64/vc16/lib
cmake --build build --config Release
```

如果 OpenCV 安装在其他位置，请设置 `-DHAND_EYE_OPENCV_DIR=...` 或 `-DOpenCV_DIR=...`。

## CSV 输入

训练数据和独立验证数据均可通过界面导入 CSV。每行支持：

```text
id, gripper_tx, gripper_ty, gripper_tz, gripper_r1, gripper_r2, gripper_r3, target_tx, target_ty, target_tz, target_r1, target_r2, target_r3
```

旋转列数量为 3 时表示 Rodrigues、Euler XYZ 或 RPY；Quaternion 模式使用 4 个旋转值。导入前在界面选择姿态格式、角度单位、长度单位和 Pose Adapter。

内部统一采用：

```text
gripper_to_base + target_to_camera → camera_to_gripper
```

## 冒烟测试

Debug 构建后直接运行程序会执行合成数据冒烟测试，检查：

- 五种算法是否返回结果；
- 推荐矩阵与合成真值的误差；
- degree/mm 输入归一化；
- 独立验证、异常样本、相对运动退化检测；
- JSON/YAML/TXT/C++/Python 导出与 CSV/JSON 读回。

## 目录结构

```text
src/
  core/       标定服务、姿态转换、数据验证、合成数据
  domain/     数据模型、算法和方向定义
  io/         CSV/JSON/YAML/代码导出、Pose Adapter
  models/     Qt 表格模型
  mainwindow  Qt 6 用户界面
docs/plans/   功能设计与实施计划
```
