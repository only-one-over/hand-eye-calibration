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
- 自主相机内参标定：多选棋盘格图片，自动计算普通针孔模型、5 个畸变参数和重投影误差。
- 相机标定质量检查：至少 6 张有效图片、统一分辨率、单图 RMSE 异常剔除和覆盖性提示。
- 手动 TCP/相机位姿标定：逐组输入 `gripper → base` 和 `target → camera`，无需图片或 PnP 即可复用五种手眼算法。
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

## 图片配对输入

完整标定流程是一组机器人末端位姿对应一张同一时刻的标定板图片：

```text
robot_pose_i + calibration_image_i
        ↓
棋盘格角点检测
        ↓
solvePnP + 相机内参
        ↓
target → camera
        ↓
手眼标定
```

通过“导入机器人位姿 + 图片 CSV”加载配对样本，格式为：

```text
id,image_path,tx,ty,tz,r1,r2,r3
```

Quaternion 输入时增加第 4 个旋转值：

```text
id,image_path,tx,ty,tz,qw,qx,qy,qz
```

界面中填写棋盘格内角点数量和方格尺寸。通过“相机内参”分页多选 10～30 张不同姿态的棋盘格图片，程序使用 OpenCV `calibrateCamera` 自动计算 3×3 相机矩阵和 `k1,k2,p1,p2,k3` 五个畸变参数。成功后内参会自动应用到当前会话，并需要重新处理手眼标定图片。每个样本会保留图片路径、检测状态、角点数量、单图 RMSE 和是否被剔除。

当前首版支持棋盘格，后续可在同一个 `BoardPoseEstimator` 接口下扩展 ArUco/ChArUco。

## Qt 界面流程

主窗口使用顶部分页：`首页 | 采集 | 参数 | 相机内参 | 手动输入 | 当前数据 | 标定结果`。

手眼标定流程：

1. 选择 Eye-In-Hand、姿态格式、单位、Pose Adapter、机器人和相机信息。
2. 点击“上传机器人坐标”，导入本轮机器人位姿 CSV。
3. 点击“上传标定板图片”，按机器人坐标顺序多选图片。
4. 两次上传都会提醒用户确认数量、编号和顺序一致。
5. 如果没有相机内参，先进入“相机内参”分页完成自主标定。
6. 点击“处理标定板图片并生成相机位姿”，再选择算法或运行五种算法推荐。

如果用户已经从机器人控制器和视觉系统获得成对位姿，可以直接进入“手动输入”分页。TCP 输入方向固定为 `gripper → base`，相机输入方向固定为 `target → camera`；旋转格式、角度单位和长度单位跟随“参数”页，应用时统一转换为 Rodrigues 弧度和米。手动数据会替换当前训练样本，校验通过后可直接执行五种算法。

“当前数据”分页逐组显示机器人坐标、图片路径、标定板计算出的或手动输入的 `target→camera` 坐标、检测状态、角点数量和 PnP RMSE；“相机内参”分页显示角点预览、单图误差和标定报告。

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
- 合成相机内参恢复、异常图片剔除、图片数量不足、分辨率不一致、手动位姿标准化与七分页 UI 检查。

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
