# 手眼标定可靠性流水线实施计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 在现有 Qt6 手眼标定程序中增加从质量检查到 Bootstrap 不确定度评估的一键可靠性流水线，并输出最终矩阵及置信度。

**Architecture:** 新增 `ReliabilityPipelineService` 作为编排层，复用现有 `CalibrationService`、`PoseQualityService`、`NonlinearOptimizer` 和 `PointCalibrationService`。控制器只负责准备数据、执行服务、写回最终样本与结果；结果页显示阶段状态、剔除样本、Huber 优化前后指标以及 Bootstrap 95% 区间。PosePairs 执行 PnP/五算法链路，FixedPoint3D 对不适用阶段显示跳过并执行点基链路。

**Tech Stack:** C++17、Qt6 Widgets/Concurrent、OpenCV calib3d/core、现有 JSON/YAML 导出。

---

### Task 1: 增加流水线报告和 Bootstrap 数据模型

**Files:**
- Modify: `src/domain/calibration_types.h`
- Modify: `src/io/dataset_io.cpp`

**Steps:**

1. 增加流水线阶段状态、PnP 质量、Bootstrap 报告和总流水线报告结构。
2. 在 `CalibrationDataset` 增加流水线报告和 Bootstrap 次数/置信度配置。
3. 增加 JSON/YAML 序列化字段，兼容没有新字段的旧文件。
4. 运行编译，确认模型和导出代码无类型错误。

### Task 2: 实现可靠性流水线服务

**Files:**
- Create: `src/core/reliability_pipeline_service.h`
- Create: `src/core/reliability_pipeline_service.cpp`
- Modify: `CMakeLists.txt`

**Steps:**

1. 实现运动激励检查，复用 `validateDataset` 并记录最大相对旋转和轴覆盖。
2. 实现图片样本 PnP RMSE/检测状态统计；手动位姿或 FixedPoint3D 明确报告为跳过。
3. 调用五种 OpenCV 算法或点基算法，记录推荐结果和 AX=XB 一致性指标。
4. 计算 Fixed Target/Fixed Point 一致性，根据样本残差自动剔除异常样本并最多重算一次。
5. 调用现有 Huber 非线性精修，标记优化前后 RMSE 和异常点数。
6. 对最终方法执行可复现的 Bootstrap 重采样，计算成功次数、旋转/平移标准差、95% 分量区间和总体置信度。
7. 返回最终矩阵、保留样本、阶段报告和不确定度报告。

### Task 3: 控制器编排和状态回写

**Files:**
- Modify: `src/controllers/calibration_controller.h`
- Modify: `src/controllers/calibration_controller.cpp`
- Modify: `src/mainwindow.h`
- Modify: `src/mainwindow.cpp`

**Steps:**

1. 增加 `runReliabilityPipeline()`、Bootstrap 参数更新接口和流水线信号。
2. 使用 QtConcurrent 执行耗时计算，完成后用流水线返回值替换训练样本并写回最终结果。
3. 保留独立验证数据，更新残差、推荐结果、首页状态和日志。
4. 增加结果页流水线入口及页面导航连接。

### Task 4: 结果页展示阶段和置信度

**Files:**
- Modify: `src/views/calibration_result_page.h`
- Modify: `src/views/calibration_result_page.cpp`

**Steps:**

1. 增加“一键执行完整可靠性流水线”按钮和 Bootstrap 次数输入。
2. 增加阶段状态表：运动激励、PnP、五算法、AX=XB、Fixed Target、异常剔除、Huber、Bootstrap。
3. 显示最终矩阵、剔除样本列表、旋转/平移 95% 区间、标准差和置信度。
4. 失败阶段显示具体原因，跳过阶段显示适用范围。

### Task 5: 验证和回归

**Files:**
- Modify: `tests` or existing smoke-test source discovered in the repository
- Modify: `README.md`

**Steps:**

1. 增加合成数据流水线测试，确认最终矩阵可生成且 Bootstrap 有成功重采样。
2. 增加异常样本自动剔除测试和置信区间往返导出测试。
3. 验证 FixedPoint3D 阶段跳过规则和点基 Bootstrap。
4. 执行 Debug 构建、现有 smoke test 和 UI smoke test。
5. 更新 README 流程说明。

