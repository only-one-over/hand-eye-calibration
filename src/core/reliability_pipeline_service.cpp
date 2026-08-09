#include "core/reliability_pipeline_service.h"

#include "core/calibration_service.h"
#include "core/dataset_validator.h"
#include "core/matrix_utils.h"
#include "core/nonlinear_optimizer.h"
#include "core/point_calibration_service.h"
#include "core/pose_quality_service.h"

#include <QElapsedTimer>
#include <QRandomGenerator>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace handeye {

namespace {

constexpr double kPi = 3.14159265358979323846;

void addStage(ReliabilityPipelineReport *report, const QString &name,
              PipelineStageState state, const QString &message)
{
    if (!report) return;
    report->stages.append({name, state, message});
}

int sampleCount(const CalibrationDataset &dataset)
{
    return dataset.inputMode == CalibrationInputMode::FixedPoint3D
               ? dataset.pointSamples.size()
               : dataset.samples.size();
}

CalibrationResult chooseResult(const QVector<CalibrationResult> &results)
{
    for (const CalibrationResult &result : results)
        if (result.recommended && result.success) return result;
    for (const CalibrationResult &result : results)
        if (result.success) return result;
    return {};
}

QVector<CalibrationResult> calculateResults(const CalibrationDataset &dataset)
{
    if (dataset.inputMode == CalibrationInputMode::FixedPoint3D)
        return {PointCalibrationService::calibrate(dataset)};
    return CalibrationService::calibrateAll(dataset);
}

PnpQualityReport evaluatePnp(const CalibrationDataset &dataset)
{
    PnpQualityReport report;
    report.available = dataset.inputMode == CalibrationInputMode::PosePairs;
    if (!report.available) return report;

    double sum = 0.0;
    for (const PoseSample &sample : dataset.samples) {
        if (sample.imagePath.trimmed().isEmpty()) continue;
        ++report.totalImageSamples;
        if (sample.imageStatus != ImageSampleStatus::PoseEstimated
            || !std::isfinite(sample.pnpReprojectionRmsePx)) {
            report.warnings << QStringLiteral("样本 %1 没有有效的 PnP 位姿。").arg(sample.id);
            continue;
        }
        ++report.validSamples;
        sum += sample.pnpReprojectionRmsePx;
        report.maxRmsePx = std::max(report.maxRmsePx, sample.pnpReprojectionRmsePx);
        if (sample.pnpReprojectionRmsePx > report.thresholdPx) ++report.outlierCount;
    }
    report.meanRmsePx = report.validSamples > 0 ? sum / report.validSamples : 0.0;
    report.passed = report.totalImageSamples == 0
                    || (report.validSamples == report.totalImageSamples && report.outlierCount == 0);
    if (report.totalImageSamples == 0)
        report.warnings << QStringLiteral("当前样本没有图片 PnP 数据，PnP 质量检查跳过。");
    else if (!report.passed)
        report.warnings << QStringLiteral("PnP 有效样本 %1/%2，超过 %3 px 的样本 %4 个。")
                            .arg(report.validSamples).arg(report.totalImageSamples)
                            .arg(report.thresholdPx, 0, 'f', 2).arg(report.outlierCount);
    return report;
}

QSet<int> outlierIds(const CalibrationDataset &dataset, const CalibrationResult &result,
                     const PnpQualityReport &pnp)
{
    QSet<int> ids;
    if (dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
        for (const FixedPointSample &sample : result.fixedPointReport.samples)
            if (sample.outlier) ids.insert(sample.sampleId);
        return ids;
    }
    for (const SampleResidual &sample : result.trainingReport.sampleResiduals)
        if (sample.outlier) ids.insert(sample.sampleId);
    if (pnp.available) {
        for (const PoseSample &sample : dataset.samples) {
            if (sample.imageStatus == ImageSampleStatus::PoseEstimated
                && sample.pnpReprojectionRmsePx > pnp.thresholdPx)
                ids.insert(sample.id);
        }
    }
    return ids;
}

void removeOutliers(CalibrationDataset *dataset, const QSet<int> &ids)
{
    if (!dataset || ids.isEmpty()) return;
    if (dataset->inputMode == CalibrationInputMode::FixedPoint3D) {
        QVector<PointSample> kept;
        for (const PointSample &sample : std::as_const(dataset->pointSamples))
            if (!ids.contains(sample.id)) kept.append(sample);
        dataset->pointSamples = kept;
        return;
    }
    QVector<PoseSample> kept;
    for (const PoseSample &sample : std::as_const(dataset->samples))
        if (!ids.contains(sample.id)) kept.append(sample);
    dataset->samples = kept;
}

double percentile(QVector<double> values, double probability)
{
    if (values.isEmpty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = std::clamp(probability, 0.0, 1.0)
                            * static_cast<double>(values.size() - 1);
    const int lower = static_cast<int>(std::floor(position));
    const int upper = static_cast<int>(std::ceil(position));
    if (lower == upper) return values.at(lower);
    const double fraction = position - lower;
    return values.at(lower) * (1.0 - fraction) + values.at(upper) * fraction;
}

BootstrapReport bootstrap(const CalibrationDataset &dataset, const CalibrationResult &finalResult,
                          int requested, double confidenceLevel)
{
    BootstrapReport report;
    report.available = true;
    report.requestedResamples = std::max(0, requested);
    report.confidenceLevel = std::clamp(confidenceLevel, 0.5, 0.999);
    if (report.requestedResamples <= 0) {
        report.message = QStringLiteral("Bootstrap 重采样次数为 0，未执行。");
        return report;
    }

    const int count = sampleCount(dataset);
    if (count < 5) {
        report.message = QStringLiteral("Bootstrap 至少需要 5 组有效样本。");
        report.warnings << report.message;
        return report;
    }

    CalibrationMethod method = finalResult.method;
    if (method == CalibrationMethod::Nonlinear)
        method = CalibrationMethod::Tsai;
    const cv::Matx44d reference = matrix::toMat(finalResult.cameraToGripper);
    const Vector3 referenceTranslation{reference(0, 3), reference(1, 3), reference(2, 3)};
    QVector<Vector3> rotationSamples;
    QVector<Vector3> translationSamples;
    rotationSamples.reserve(report.requestedResamples);
    translationSamples.reserve(report.requestedResamples);
    QRandomGenerator generator(0x51E123u);

    for (int iteration = 0; iteration < report.requestedResamples; ++iteration) {
        CalibrationDataset resampled = dataset;
        if (dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
            resampled.pointSamples.clear();
            for (int index = 0; index < count; ++index) {
                PointSample sample = dataset.pointSamples.at(static_cast<int>(generator.bounded(count)));
                sample.id = index + 1;
                resampled.pointSamples.append(sample);
            }
        } else {
            resampled.samples.clear();
            for (int index = 0; index < count; ++index) {
                PoseSample sample = dataset.samples.at(static_cast<int>(generator.bounded(count)));
                sample.id = index + 1;
                resampled.samples.append(sample);
            }
        }

        CalibrationResult sampled;
        if (dataset.inputMode == CalibrationInputMode::FixedPoint3D)
            sampled = PointCalibrationService::calibrate(resampled);
        else
            sampled = CalibrationService::calibrate(resampled, method);
        if (!sampled.success) continue;

        const cv::Matx44d pose = matrix::toMat(sampled.cameraToGripper);
        const Vector3 rotation = matrix::toRodrigues(reference.get_minor<3, 3>(0, 0).t()
                                                      * pose.get_minor<3, 3>(0, 0));
        rotationSamples.append({rotation[0] * 180.0 / kPi, rotation[1] * 180.0 / kPi,
                                rotation[2] * 180.0 / kPi});
        translationSamples.append({pose(0, 3) - referenceTranslation[0],
                                   pose(1, 3) - referenceTranslation[1],
                                   pose(2, 3) - referenceTranslation[2]});
    }

    report.successfulResamples = rotationSamples.size();
    const int required = std::max(1, static_cast<int>(std::ceil(report.requestedResamples * 0.8)));
    report.success = report.successfulResamples >= required;
    if (rotationSamples.isEmpty()) {
        report.message = QStringLiteral("Bootstrap 没有成功重采样。");
        report.warnings << report.message;
        return report;
    }

    QVector<double> rotationAxes[3];
    QVector<double> translationAxes[3];
    double rotationNormSquared = 0.0;
    double translationNormSquared = 0.0;
    for (int index = 0; index < rotationSamples.size(); ++index) {
        const Vector3 &rotation = rotationSamples.at(index);
        const Vector3 &translation = translationSamples.at(index);
        for (int axis = 0; axis < 3; ++axis) {
            rotationAxes[axis].append(rotation[axis]);
            translationAxes[axis].append(translation[axis]);
        }
        rotationNormSquared += rotation[0] * rotation[0] + rotation[1] * rotation[1]
                               + rotation[2] * rotation[2];
        translationNormSquared += translation[0] * translation[0] + translation[1] * translation[1]
                                  + translation[2] * translation[2];
    }
    const double inverseCount = 1.0 / static_cast<double>(rotationSamples.size());
    for (int axis = 0; axis < 3; ++axis) {
        auto computeStd = [inverseCount](const QVector<double> &values) {
            double mean = 0.0;
            for (double value : values) mean += value;
            mean *= inverseCount;
            double squared = 0.0;
            for (double value : values) squared += (value - mean) * (value - mean);
            return std::sqrt(squared * inverseCount);
        };
        report.rotationStdDeg[axis] = computeStd(rotationAxes[axis]);
        report.translationStdM[axis] = computeStd(translationAxes[axis]);
        const double alpha = (1.0 - report.confidenceLevel) * 0.5;
        report.rotationLowerDeg[axis] = percentile(rotationAxes[axis], alpha);
        report.rotationUpperDeg[axis] = percentile(rotationAxes[axis], 1.0 - alpha);
        report.translationLowerM[axis] = percentile(translationAxes[axis], alpha);
        report.translationUpperM[axis] = percentile(translationAxes[axis], 1.0 - alpha);
    }
    report.rotationNormStdDeg = std::sqrt(rotationNormSquared * inverseCount);
    report.translationNormStdM = std::sqrt(translationNormSquared * inverseCount);
    report.confidenceScore = 100.0 * static_cast<double>(report.successfulResamples)
                             / static_cast<double>(report.requestedResamples);
    report.message = QStringLiteral("Bootstrap 完成：%1/%2 次成功，置信水平 %3%。")
                         .arg(report.successfulResamples).arg(report.requestedResamples)
                         .arg(report.confidenceLevel * 100.0, 0, 'f', 1);
    if (!report.success)
        report.warnings << QStringLiteral("成功重采样比例低于 80%，不确定度仅供参考。");
    return report;
}

} // namespace

ReliabilityPipelineExecution ReliabilityPipelineService::run(const CalibrationDataset &dataset,
                                                              int bootstrapResamples,
                                                              double confidenceLevel)
{
    ReliabilityPipelineExecution execution;
    execution.refinedDataset = dataset;
    execution.refinedDataset.reliabilityPipelineReport = {};
    execution.report.available = true;
    execution.report.initialSampleCount = sampleCount(dataset);
    QElapsedTimer timer;
    timer.start();

    if (dataset.mode == CalibrationMode::EyeToHand) {
        execution.report.errors << QStringLiteral("Eye-To-Hand 当前仍未完成，可靠性流水线已停止。");
        addStage(&execution.report, QStringLiteral("运动激励检查"), PipelineStageState::Failed,
                 execution.report.errors.last());
        execution.report.message = execution.report.errors.last();
        execution.report.elapsedMs = timer.elapsed();
        return execution;
    }
    if (sampleCount(dataset) < 5) {
        execution.report.errors << QStringLiteral("可靠性流水线至少需要 5 组样本。");
        addStage(&execution.report, QStringLiteral("运动激励检查"), PipelineStageState::Failed,
                 execution.report.errors.last());
        execution.report.message = execution.report.errors.last();
        execution.report.elapsedMs = timer.elapsed();
        return execution;
    }

    CalibrationDataset working = dataset;
    const PoseQualityReport initialQuality = working.inputMode == CalibrationInputMode::FixedPoint3D
                                                 ? PoseQualityService::evaluatePointQuality(working)
                                                 : PoseQualityService::evaluatePoseQuality(working);
    execution.report.qualityReport = initialQuality;
    if (!initialQuality.calculable) {
        execution.report.errors << initialQuality.warnings;
        addStage(&execution.report, QStringLiteral("运动激励检查"), PipelineStageState::Failed,
                 initialQuality.warnings.join(' '));
        execution.report.message = QStringLiteral("运动激励不足或退化，无法执行可靠性流水线。");
        execution.report.elapsedMs = timer.elapsed();
        return execution;
    }
    addStage(&execution.report, QStringLiteral("运动激励检查"), PipelineStageState::Passed,
             QStringLiteral("样本 %1，最大相对旋转 %2°，独立旋转轴 %3。")
                 .arg(sampleCount(working)).arg(initialQuality.maxRelativeRotationDeg, 0, 'f', 2)
                 .arg(initialQuality.independentAxisCount));

    execution.report.pnpReport = evaluatePnp(working);
    if (!execution.report.pnpReport.available) {
        addStage(&execution.report, QStringLiteral("PnP 质量检查"), PipelineStageState::Skipped,
                 QStringLiteral("FixedPoint3D 不需要相机旋转和图片 PnP。"));
    } else if (!execution.report.pnpReport.passed) {
        addStage(&execution.report, QStringLiteral("PnP 质量检查"), PipelineStageState::Warning,
                 execution.report.pnpReport.warnings.join(' '));
        execution.report.warnings += execution.report.pnpReport.warnings;
    } else {
        addStage(&execution.report, QStringLiteral("PnP 质量检查"),
                 execution.report.pnpReport.totalImageSamples > 0 ? PipelineStageState::Passed
                                                                    : PipelineStageState::Skipped,
                 execution.report.pnpReport.totalImageSamples > 0
                     ? QStringLiteral("PnP RMSE 平均 %1 px，最大 %2 px。")
                           .arg(execution.report.pnpReport.meanRmsePx, 0, 'f', 3)
                           .arg(execution.report.pnpReport.maxRmsePx, 0, 'f', 3)
                     : QStringLiteral("没有图片 PnP 样本。"));
    }

    QVector<CalibrationResult> rawResults = calculateResults(working);
    CalibrationResult best = chooseResult(rawResults);
    if (!best.success) {
        execution.report.errors << QStringLiteral("五算法/点基算法均未成功求解。");
        addStage(&execution.report, QStringLiteral("五算法计算"), PipelineStageState::Failed,
                 execution.report.errors.last());
        execution.report.message = execution.report.errors.last();
        execution.report.elapsedMs = timer.elapsed();
        return execution;
    }
    addStage(&execution.report, QStringLiteral("五算法计算"),
             best.trainingReport.passed ? PipelineStageState::Passed : PipelineStageState::Warning,
             QStringLiteral("推荐方法：%1，旋转 RMSE %2°，平移 RMSE %3 m。")
                 .arg(methodName(best.method)).arg(best.trainingReport.rotationRmseDeg, 0, 'f', 5)
                 .arg(best.trainingReport.translationRmseM, 0, 'f', 7));

    if (working.inputMode == CalibrationInputMode::FixedPoint3D) {
        execution.report.fixedPointReport = best.fixedPointReport;
        addStage(&execution.report, QStringLiteral("Fixed Target 一致性"),
                 best.fixedPointReport.success && best.fixedPointReport.outlierCount == 0
                     ? PipelineStageState::Passed : PipelineStageState::Warning,
                 QStringLiteral("FixedPoint3D 固定点 RMSE %1 m，最大 %2 m。")
                     .arg(best.fixedPointReport.rmseM, 0, 'f', 7)
                     .arg(best.fixedPointReport.maxErrorM, 0, 'f', 7));
        execution.report.axXbReport = best.trainingReport;
        addStage(&execution.report, QStringLiteral("AX=XB 一致性"), PipelineStageState::Skipped,
                 QStringLiteral("FixedPoint3D 不使用 AX=XB 位姿对方程。"));
    } else {
        execution.report.axXbReport = best.trainingReport;
        addStage(&execution.report, QStringLiteral("AX=XB 一致性"),
                 best.trainingReport.passed ? PipelineStageState::Passed : PipelineStageState::Warning,
                 QStringLiteral("旋转 RMSE %1°，平移 RMSE %2 m，异常样本 %3。")
                     .arg(best.trainingReport.rotationRmseDeg, 0, 'f', 5)
                     .arg(best.trainingReport.translationRmseM, 0, 'f', 7)
                     .arg(best.trainingReport.outlierCount));
        execution.report.fixedTargetReport = best.fixedTargetReport;
        addStage(&execution.report, QStringLiteral("Fixed Target 一致性"),
                 best.fixedTargetReport.success && best.fixedTargetReport.outlierCount == 0
                     ? PipelineStageState::Passed : PipelineStageState::Warning,
                 QStringLiteral("旋转 RMSE %1°，平移 RMSE %2 m，异常样本 %3。")
                     .arg(best.fixedTargetReport.rotationRmseDeg, 0, 'f', 5)
                     .arg(best.fixedTargetReport.translationRmseM, 0, 'f', 7)
                     .arg(best.fixedTargetReport.outlierCount));
    }

    const QSet<int> candidates = outlierIds(working, best, execution.report.pnpReport);
    const int minimumSamples = working.inputMode == CalibrationInputMode::FixedPoint3D ? 5 : 5;
    if (!candidates.isEmpty() && sampleCount(working) - candidates.size() >= minimumSamples) {
        for (int id : candidates) execution.report.removedSampleIds.append(id);
        execution.report.autoRemovedCount = candidates.size();
        removeOutliers(&working, candidates);
        rawResults = calculateResults(working);
        best = chooseResult(rawResults);
        QStringList removedIds;
        for (int id : execution.report.removedSampleIds) removedIds.append(QString::number(id));
        addStage(&execution.report, QStringLiteral("异常样本自动剔除"), PipelineStageState::Passed,
                 QStringLiteral("已剔除 %1 个样本并完成重算：%2。")
                     .arg(candidates.size()).arg(removedIds.join(QStringLiteral(", "))));
    } else {
        if (!candidates.isEmpty())
            execution.report.warnings << QStringLiteral("检测到异常样本，但剔除后样本数不足，保留原数据。");
        addStage(&execution.report, QStringLiteral("异常样本自动剔除"),
                 candidates.isEmpty() ? PipelineStageState::Passed : PipelineStageState::Warning,
                 candidates.isEmpty() ? QStringLiteral("未检测到需要剔除的样本。")
                                       : QStringLiteral("异常样本未剔除：保留至少 5 组样本。"));
    }

    if (!best.success) {
        execution.report.errors << QStringLiteral("剔除异常样本后无法重新求解。");
        execution.report.message = execution.report.errors.last();
        execution.report.elapsedMs = timer.elapsed();
        return execution;
    }

    CalibrationResult finalResult = best;
    if (working.inputMode == CalibrationInputMode::PosePairs) {
        finalResult = NonlinearOptimizer::refinePose(working, best);
        addStage(&execution.report, QStringLiteral("归一化 Huber 优化"),
                 finalResult.success ? PipelineStageState::Passed : PipelineStageState::Failed,
                 finalResult.optimizationReport.message.isEmpty()
                     ? finalResult.message : finalResult.optimizationReport.message);
    } else {
        execution.report.optimizationReport = best.optimizationReport;
        addStage(&execution.report, QStringLiteral("归一化 Huber 优化"),
                 best.optimizationReport.success ? PipelineStageState::Passed : PipelineStageState::Warning,
                 best.optimizationReport.message.isEmpty()
                     ? QStringLiteral("点基 Huber 优化已执行。") : best.optimizationReport.message);
    }
    if (!finalResult.success) {
        execution.report.errors << QStringLiteral("Huber 优化未生成有效最终矩阵。");
        execution.report.message = execution.report.errors.last();
        execution.report.elapsedMs = timer.elapsed();
        return execution;
    }

    execution.report.qualityReport = working.inputMode == CalibrationInputMode::FixedPoint3D
                                         ? PoseQualityService::evaluatePointQuality(working)
                                         : PoseQualityService::evaluatePoseQuality(working);
    execution.report.finalMethod = finalResult.method;
    execution.report.finalCameraToGripper = finalResult.cameraToGripper;
    execution.report.optimizationReport = finalResult.optimizationReport;
    execution.report.fixedTargetReport = finalResult.fixedTargetReport;
    execution.report.fixedPointReport = finalResult.fixedPointReport;
    execution.report.axXbReport = finalResult.trainingReport;

    finalResult.bootstrapReport = bootstrap(working, finalResult, bootstrapResamples, confidenceLevel);
    execution.report.bootstrapReport = finalResult.bootstrapReport;
    addStage(&execution.report, QStringLiteral("Bootstrap 重采样"),
             finalResult.bootstrapReport.success ? PipelineStageState::Passed : PipelineStageState::Warning,
             finalResult.bootstrapReport.message);

    finalResult.qualityReport = execution.report.qualityReport;
    finalResult.recommended = true;
    QVector<CalibrationResult> storedResults;
    for (CalibrationResult result : rawResults) {
        result.recommended = false;
        storedResults.append(result);
    }
    if (working.inputMode == CalibrationInputMode::FixedPoint3D)
        storedResults.clear();
    storedResults.append(finalResult);

    execution.refinedDataset = working;
    execution.refinedDataset.results = storedResults;
    execution.refinedDataset.bootstrapResamples = bootstrapResamples;
    execution.refinedDataset.bootstrapConfidence = confidenceLevel;
    execution.report.finalSampleCount = sampleCount(working);
    execution.report.success = true;
    execution.report.passed = finalResult.trainingReport.passed
                              && finalResult.bootstrapReport.success
                              && execution.report.errors.isEmpty();
    execution.report.message = execution.report.passed
                                  ? QStringLiteral("可靠性流水线完成，最终结果通过。")
                                  : QStringLiteral("可靠性流水线完成，但存在警告，请查看阶段报告。");
    execution.report.warnings += finalResult.bootstrapReport.warnings;
    execution.report.completedAt = QDateTime::currentDateTime();
    execution.report.elapsedMs = timer.elapsed();
    execution.refinedDataset.reliabilityPipelineReport = execution.report;
    execution.finalResult = finalResult;
    return execution;
}

} // namespace handeye
