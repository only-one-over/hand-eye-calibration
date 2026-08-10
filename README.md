# Qt6 手眼标定工具

基于 Qt 6、OpenCV 和 Eigen 的手眼标定桌面工具，面向“输入标准化 → 标定可靠性 → 数据验证 → 工程导出”的使用流程。

## 已支持功能

- 五种 OpenCV 手眼标定算法：Tsai-Lenz、Park-Martin、Horaud、Andreff、Daniilidis。
- 自动计算全部算法，并按可靠性报告推荐结果。
- 输入姿态格式：Rodrigues、Euler XYZ、RPY、Quaternion（WXYZ）。
- 角度单位：degree/rad；长度单位：mm/m，导入时统一转换为内部的 rad/m。
- 明确记录并展示方向：机器人 `gripper → base`、相机 `target → camera`。
- 已启用 Eye-To-Hand：PosePairs 输出 `camera → base` 和 `target → gripper`；FixedPoint3D 输出 `camera → base` 和 TCP 上特征点坐标。
- 相对运动退化检测：样本数量、重复样本、旋转激励、旋转轴分布和有限值检查。
- 真实 RMSE、平均误差、最大误差、单样本残差和异常样本标记。
- 支持独立验证数据集，并输出是否通过验证。
- 合成真值测试，验证矩阵方向、数值误差、单位归一化、异常检测和退化检测。
- 机器人 Pose Adapter：Generic、Universal Robots、KUKA、FANUC。
- 自主相机内参标定：多选棋盘格图片，自动计算普通针孔模型、5 个畸变参数和重投影误差。
- 相机标定质量检查：至少 6 张有效图片、统一分辨率、单图 RMSE 异常剔除和覆盖性提示。
- 手动输入支持两种模式：PosePairs 逐组输入 `gripper → base` 与 `target → camera`；FixedPoint3D 输入 TCP 6D 与相机 XYZ，不需要相机旋转。
- Eye-To-Hand PosePairs 使用 OpenCV Robot-World Shah、Li 和非线性精修；点基模式要求特征点刚性安装在 TCP 上，不能使用工作台固定点。
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

主窗口使用顶部分页：`首页 | 采集 | 参数 | 相机内参 | 手动输入 | 标定板 PDF | 当前数据 | 标定结果`。

手眼标定流程：

1. 在“参数”页选择 Eye-In-Hand 或 Eye-To-Hand，并选择 PosePairs / FixedPoint3D、姿态格式、单位、Pose Adapter、机器人和相机信息。
2. 点击“上传机器人坐标”，导入本轮机器人位姿 CSV。
3. 点击“上传标定板图片”，按机器人坐标顺序多选图片。
4. 两次上传都会提醒用户确认数量、编号和顺序一致。
5. 如果没有相机内参，先进入“相机内参”分页完成自主标定。
6. 点击“处理标定板图片并生成相机位姿”，再选择当前模式对应的算法并运行推荐计算。

如果用户已经从机器人控制器和视觉系统获得数据，可以直接进入“手动输入”分页。PosePairs 中 TCP 输入方向固定为 `gripper → base`，相机输入方向固定为 `target → camera`；FixedPoint3D 中相机只输入 XYZ。旋转格式、角度单位和长度单位跟随“参数”页，应用时统一转换为 Rodrigues 弧度和米。手动数据会替换当前训练样本，校验通过后按当前模式执行算法。

“当前数据”分页逐组显示机器人坐标、图片路径、标定板计算出的或手动输入的 `target→camera` 坐标、检测状态、角点数量和 PnP RMSE；“相机内参”分页显示角点预览、单图误差和标定报告。

## CSV 输入

训练数据和独立验证数据均可通过界面导入 CSV。每行支持：

```text
id, gripper_tx, gripper_ty, gripper_tz, gripper_r1, gripper_r2, gripper_r3, target_tx, target_ty, target_tz, target_r1, target_r2, target_r3
```

旋转列数量为 3 时表示 Rodrigues、Euler XYZ 或 RPY；Quaternion 模式使用 4 个旋转值。导入前在界面选择姿态格式、角度单位、长度单位和 Pose Adapter。

Eye-In-Hand 内部统一采用：

```text
gripper_to_base + target_to_camera → camera_to_gripper
```

Eye-To-Hand 使用不同的方程，不能直接套用上面的输出方向：

```text
Aᵢ = T_base_gripper(i), Bᵢ = T_camera_target(i)
Aᵢ × T_gripper_target = T_base_camera × Bᵢ
输出：camera_to_base、target_to_gripper
```

点基 Eye-To-Hand 使用：

```text
T_base_gripper(i) × p_gripper = T_base_camera × p_camera(i)
```

这里的 `p_gripper` 是特征点在 TCP 坐标系中的固定位置。每组 `p_camera` 必须是同一个随 TCP 运动的物理点。

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

## FixedPoint3D、质量评分与检测升级

除了 PosePairs（TCP 6D + target→camera 6D），当前版本还支持 FixedPoint3D：每组输入机器人 TCP 的 `gripper→base` 6D 位姿，以及相机坐标系中同一个固定物理点的 `Xc,Yc,Zc`。点基模式不需要相机 `rx/ry/rz`，程序会联合求解 `camera→gripper` 和 base 固定点。

结果页提供 fixed target pose、每个 sample 到鲁棒均值或指定 reference 的误差、Huber 非线性精修、优化前后 RMSE，以及由样本数量、旋转幅度、旋转轴和空间分布组成的 Pose Quality Score。

标定板支持 Chessboard（`findChessboardCornersSB` 优先、Classic 回退）、ChArUco 和 ArUco Grid。平面 PnP 会比较 `SOLVEPNP_ITERATIVE` 与 `SOLVEPNP_IPPE`，并保存检测器、PnP 方法和误差摘要。
## 可靠性流水线与诊断

完整流水线顺序为：

`运动激励检查 → PnP 质量检查 → 原始算法 → AX=XB/AX=YB 一致性 → Fixed Target（仅 Eye-In-Hand）→ 单样本异常验证 → 归一化 Huber → Bootstrap`。

所有姿态在内部统一为 Rodrigues 弧度和米。Eye-In-Hand 导出 `camera→gripper`；Eye-To-Hand 导出 `camera→base`，并同时给出 `target→gripper` 或 TCP 上特征点坐标。

Nonlinear 精修后会重新独立计算 AX=XB 报告和 Fixed Target 报告，二者不会互相覆盖。归一化 Huber 使用 1° 旋转尺度和 1 mm 平移尺度。Bootstrap 的 `successRate` 始终为 `[0,1]` 范围内的成功重采样比例；`confidenceLevel` 只表示置信区间水平。

5～7 组样本使用“保留全部原始样本 + 随机补充重复样本”的小样本稳定模式，仍输出成功率，但 `uncertaintyReliable=false`，不确定度只作为参考。8 组及以上才使用普通有放回 Bootstrap。样本少于 5 组时不会执行 Bootstrap。

Eye-To-Hand FixedPoint3D 会对 `3N×15` 线性初值系统执行 SVD 诊断，结果页和导出文件包含 `rank`、`condition number`、满秩状态及条件数是否不超过 `1e8`。秩不足会阻止点基求解，条件数过高会保留结果但标记为未通过。

修改 Pose Adapter、旋转格式、角度单位、长度单位或姿态约定后，程序保留旧样本供查看，但锁定计算并提示“必须重新导入原始机器人数据”。重新导入机器人 CSV、配对 CSV 或重新应用手动数据后才会解除锁定；仅修改机器人/相机名称不会触发该机制。

本地测试包括固定随机种子的 100 轮 Monte-Carlo：每轮包含 0.5 mm/0.0005 rad 机器人噪声、0.8 mm/0.001 rad PnP 噪声和约 10% 大异常点，并验证原始算法、Nonlinear 精修和完整可靠性流水线。

Run the local checks after configuring the project:

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

## Calibration board PDF and fixed documents

The Board PDF page generates the current board from `BoardSpec` without changing the
calibration mathematics. Chessboard, ChArUco and ArUco Grid are supported. The custom-size
PDF keeps the board physical size; the A4 version uses 1:1 tiled pages. Print at 100% and
verify the included 100 mm scale line with a ruler.

Built-in PDF instructions are bundled as Qt resources. Files with the same names in the
application `docs/` directory take precedence, so site-specific instructions can be replaced
without recompiling the program. The Help menu and Board PDF page can open the documents.

Generated board PDFs are saved by default to the user documents directory:
`HandEyeCalibration/board_pdfs`. The directory is created on first use. The generated filename
contains the board type, current dimensions, output mode and timestamp; existing files are never
overwritten. If a PDF with the same board specification and output mode already exists, the
Generate button reuses the newest matching file instead of rendering it again. Use the separate
Board PDF page's `另存` buttons when a different output location is needed.
