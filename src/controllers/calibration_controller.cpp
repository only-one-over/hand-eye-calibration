#include "controllers/calibration_controller.h"

#include "core/board_pose_estimator.h"
#include "core/camera_calibration_service.h"
#include "core/calibration_service.h"
#include "core/dataset_validator.h"
#include "core/matrix_utils.h"
#include "core/nonlinear_optimizer.h"
#include "core/point_calibration_service.h"
#include "core/pose_conversion.h"
#include "core/pose_quality_service.h"
#include "core/synthetic_dataset.h"
#include "io/dataset_io.h"
#include "io/image_sample_io.h"

#include <QFutureWatcher>
#include <QSet>
#include <QtConcurrent>

#include <cmath>
#include <utility>

namespace handeye {

CalibrationController::CalibrationController(QObject *parent) : QObject(parent) {}

const CalibrationDataset &CalibrationController::dataset() const
{
    return m_dataset;
}

void CalibrationController::updateInputSpec(const PoseInputSpec &spec, const QString &robot,
                                             const QString &camera)
{
    m_dataset.inputSpec = spec;
    m_dataset.inputSpec.direction = PoseDirection::GripperToBase;
    m_dataset.robotName = robot;
    m_dataset.cameraName = camera;
}

void CalibrationController::updateImageProcessing(const BoardSpec &board,
                                                   const CameraIntrinsics &intrinsics)
{
    const bool boardChanged = m_dataset.boardSpec.pattern != board.pattern
                              || m_dataset.boardSpec.chessboardDetector != board.chessboardDetector
                              || m_dataset.boardSpec.pnpMethod != board.pnpMethod
                              || m_dataset.boardSpec.innerCornersX != board.innerCornersX
                              || m_dataset.boardSpec.innerCornersY != board.innerCornersY
                              || std::abs(m_dataset.boardSpec.squareSizeM - board.squareSizeM) > 1e-12
                              || m_dataset.boardSpec.arucoDictionary != board.arucoDictionary
                              || m_dataset.boardSpec.markerCountX != board.markerCountX
                              || m_dataset.boardSpec.markerCountY != board.markerCountY
                              || std::abs(m_dataset.boardSpec.markerSizeM - board.markerSizeM) > 1e-12;
    m_dataset.boardSpec = board;
    m_dataset.cameraIntrinsics = intrinsics;
    m_dataset.results.clear();
    if (boardChanged) {
        m_dataset.cameraCalibrationReport = {};
        emit cameraCalibrationChanged(m_dataset.cameraCalibrationReport);
    }
}

void CalibrationController::updateReliabilityThresholds(double rotationRmseDeg, double translationRmseM)
{
    if (rotationRmseDeg > 0.0) m_dataset.passRotationRmseDeg = rotationRmseDeg;
    if (translationRmseM > 0.0) m_dataset.passTranslationRmseM = translationRmseM;
    m_dataset.results.clear();
}

void CalibrationController::emitDatasetChanged()
{
    emit samplesChanged(m_dataset.samples);
    emit resultsChanged(m_dataset.results);
    emit statusChanged(QStringLiteral("训练样本：%1，独立验证样本：%2")
                           .arg(m_dataset.samples.size())
                           .arg(m_dataset.validationSamples.size()));
    emit cameraCalibrationChanged(m_dataset.cameraCalibrationReport);
}

void CalibrationController::emitCameraCalibrationChanged()
{
    emit cameraCalibrationChanged(m_dataset.cameraCalibrationReport);
}

void CalibrationController::applyResiduals(const ReliabilityReport &report)
{
    for (PoseSample &sample : m_dataset.samples) {
        for (const SampleResidual &residual : report.sampleResiduals) {
            if (residual.sampleId == sample.id) {
                sample.rotationResidualDeg = residual.rotationErrorDeg;
                sample.translationResidualM = residual.translationErrorM;
                sample.outlier = residual.outlier;
                break;
            }
        }
    }
}

CalibrationResult CalibrationController::recommendedResult() const
{
    for (const CalibrationResult &result : m_dataset.results)
        if (result.recommended) return result;
    return m_dataset.results.isEmpty() ? CalibrationResult{} : m_dataset.results.first();
}

void CalibrationController::newDataset()
{
    const PoseInputSpec spec = m_dataset.inputSpec;
    m_dataset = CalibrationDataset{};
    m_dataset.inputSpec = spec;
    emitDatasetChanged();
    emit reliabilityChanged(CalibrationResult{});
    emitCameraCalibrationChanged();
    emit logMessage(QStringLiteral("已新建空数据集。"));
}

void CalibrationController::generateDemo()
{
    m_dataset = makeSyntheticDataset();
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已生成 %1 组带真值的合成数据；方向和数值可由 smoke test 验证。")
                        .arg(m_dataset.samples.size()));
}

void CalibrationController::importCameraCalibrationImages(const QStringList &paths)
{
    if (paths.isEmpty()) return;
    CameraCalibrationReport report;
    report.available = true;
    report.initialImageCount = paths.size();
    report.outlierThresholdPx = 1.0;
    for (const QString &path : paths) {
        CameraCalibrationSample sample;
        sample.imagePath = path;
        sample.status = CameraCalibrationSampleStatus::NotProcessed;
        sample.message = QStringLiteral("等待角点检测。");
        report.samples.append(sample);
    }
    report.message = QStringLiteral("已选择 %1 张相机标定图片，请开始检测角点。").arg(paths.size());
    m_dataset.cameraCalibrationReport = report;
    emitCameraCalibrationChanged();
    emit logMessage(report.message);
}

void CalibrationController::detectCameraCalibrationImages()
{
    const QStringList paths = [&] {
        QStringList result;
        for (const CameraCalibrationSample &sample : m_dataset.cameraCalibrationReport.samples)
            result.append(sample.imagePath);
        return result;
    }();
    if (paths.isEmpty()) {
        emit error(QStringLiteral("无法检测角点"), QStringLiteral("请先选择相机标定图片。"));
        return;
    }
    m_dataset.cameraCalibrationReport = CameraCalibrationService::detectImages(paths, m_dataset.boardSpec);
    m_dataset.cameraCalibrationReport.outlierThresholdPx = 1.0;
    emitCameraCalibrationChanged();
    emit statusChanged(m_dataset.cameraCalibrationReport.message);
    emit logMessage(m_dataset.cameraCalibrationReport.message);
}

void CalibrationController::calibrateCameraIntrinsics()
{
    const QStringList paths = [&] {
        QStringList result;
        for (const CameraCalibrationSample &sample : m_dataset.cameraCalibrationReport.samples)
            result.append(sample.imagePath);
        return result;
    }();
    if (paths.isEmpty()) {
        emit error(QStringLiteral("无法标定相机"), QStringLiteral("请先选择相机标定图片。"));
        return;
    }

    emit cameraCalibrationStarted();
    const CameraCalibrationOptions options{6, 1.0};
    m_dataset.cameraCalibrationReport = CameraCalibrationService::calibrateImages(paths, m_dataset.boardSpec, options);
    emitCameraCalibrationChanged();
    emit statusChanged(m_dataset.cameraCalibrationReport.message);
    emit logMessage(m_dataset.cameraCalibrationReport.message);
    if (!m_dataset.cameraCalibrationReport.success) {
        emit error(QStringLiteral("相机标定失败"),
                   m_dataset.cameraCalibrationReport.errors.join('\n'));
    } else {
        applyCameraIntrinsics();
    }
    emit cameraCalibrationFinished();
}

void CalibrationController::applyCameraIntrinsics()
{
    if (!m_dataset.cameraCalibrationReport.success || !m_dataset.cameraCalibrationReport.intrinsics.valid) {
        emit error(QStringLiteral("无法应用内参"), QStringLiteral("请先成功完成相机内参标定。"));
        return;
    }
    m_dataset.cameraIntrinsics = m_dataset.cameraCalibrationReport.intrinsics;
    for (PoseSample &sample : m_dataset.samples) {
        if (sample.imagePath.isEmpty()) continue;
        sample.targetRotation = {};
        sample.targetTranslation = {};
        sample.imageStatus = ImageSampleStatus::NotProcessed;
        sample.detectedCornerCount = 0;
        sample.pnpReprojectionRmsePx = 0.0;
        sample.imageMessage = QStringLiteral("相机内参已更新，请重新处理图片。");
    }
    m_dataset.targetPosesReady = false;
    m_dataset.results.clear();
    emitDatasetChanged();
    emit statusChanged(QStringLiteral("相机内参已自动应用，请在采集页重新处理标定板图片。"));
    emit logMessage(QStringLiteral("相机自主标定内参已应用到当前会话。"));
}

void CalibrationController::clearCameraCalibrationImages()
{
    m_dataset.cameraCalibrationReport = {};
    emitCameraCalibrationChanged();
    emit statusChanged(QStringLiteral("已清空相机内参标定图片。"));
}

void CalibrationController::importCsv(const QString &path)
{
    const IoResult result = readCsv(path, &m_dataset, m_dataset.inputSpec);
    if (!result.success) {
        emit error(QStringLiteral("导入失败"), result.error);
        return;
    }
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已导入训练 CSV：%1，已标准化为 rad/m").arg(path));
}

void CalibrationController::importPoseImageCsv(const QString &path)
{
    const IoResult result = readPoseImageCsv(path, &m_dataset, m_dataset.inputSpec);
    if (!result.success) {
        emit error(QStringLiteral("导入失败"), result.error);
        return;
    }
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已导入机器人位姿 + 标定板图片配对 CSV：%1").arg(path));
}

void CalibrationController::importRobotPoseCsv(const QString &path)
{
    const IoResult result = readRobotPoseCsv(path, &m_dataset, m_dataset.inputSpec);
    if (!result.success) {
        emit error(QStringLiteral("导入失败"), result.error);
        return;
    }
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已导入 %1 组机器人位姿，请继续上传同顺序的标定板图片。")
                        .arg(m_dataset.samples.size()));
}

void CalibrationController::importCalibrationImages(const QStringList &paths)
{
    if (m_dataset.samples.isEmpty()) {
        emit error(QStringLiteral("图片上传失败"), QStringLiteral("请先上传机器人坐标。"));
        return;
    }
    if (paths.size() != m_dataset.samples.size()) {
        emit error(QStringLiteral("图片数量不一致"),
                   QStringLiteral("机器人坐标为 %1 组，图片为 %2 张；请保持数量和顺序一致。")
                       .arg(m_dataset.samples.size()).arg(paths.size()));
        return;
    }
    for (int index = 0; index < paths.size(); ++index) {
        PoseSample &sample = m_dataset.samples[index];
        sample.imagePath = paths.at(index);
        sample.imageStatus = ImageSampleStatus::NotProcessed;
        sample.imageMessage.clear();
        sample.detectedCornerCount = 0;
        sample.pnpReprojectionRmsePx = 0.0;
    }
    m_dataset.targetPosesReady = false;
    m_dataset.results.clear();
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已上传 %1 张标定板图片，请确认其顺序与机器人坐标一致。")
                        .arg(paths.size()));
}

bool CalibrationController::processBoardImages()
{
    if (m_dataset.samples.isEmpty()) {
        emit error(QStringLiteral("无法处理图片"), QStringLiteral("请先导入机器人位姿 + 图片配对 CSV。"));
        return false;
    }
    if (m_dataset.targetPosesReady) return true;
    if (!m_dataset.cameraIntrinsics.valid) {
        emit error(QStringLiteral("无法处理图片"), QStringLiteral("请先输入有效的相机内参。"));
        return false;
    }

    int processed = 0;
    int succeeded = 0;
    for (PoseSample &sample : m_dataset.samples) {
        if (sample.imagePath.trimmed().isEmpty()) continue;
        ++processed;
        const BoardPoseEstimate estimate = BoardPoseEstimator::estimateChessboard(
            sample.imagePath, m_dataset.boardSpec, m_dataset.cameraIntrinsics);
        sample.imageStatus = estimate.status;
        sample.detectedCornerCount = estimate.detectedCornerCount;
        sample.pnpReprojectionRmsePx = estimate.reprojectionRmsePx;
        sample.imageWidth = estimate.imageWidth;
        sample.imageHeight = estimate.imageHeight;
        sample.detectionMethod = estimate.detectionMethod;
        sample.selectedPnpMethod = estimate.selectedPnpMethod;
        sample.iterativePnpRmsePx = estimate.iterativePnpRmsePx;
        sample.ippePnpRmsePx = estimate.ippePnpRmsePx;
        if (sample.imageWidth > 0 && sample.imageHeight > 0 && estimate.success
            && estimate.detectedCornerCount > 0) {
            double centerX = 0.0;
            double centerY = 0.0;
            const BoardCornerDetection detected = BoardPoseEstimator::detectChessboard(
                sample.imagePath, m_dataset.boardSpec);
            for (const Vector2 &corner : detected.corners) {
                centerX += corner[0];
                centerY += corner[1];
            }
            sample.imageCenterXNorm = centerX / estimate.detectedCornerCount / sample.imageWidth;
            sample.imageCenterYNorm = centerY / estimate.detectedCornerCount / sample.imageHeight;
        } else {
            sample.imageCenterXNorm = 0.5;
            sample.imageCenterYNorm = 0.5;
        }
        sample.imageMessage = estimate.message;
        if (estimate.success) {
            sample.targetRotation = estimate.targetRotation;
            sample.targetTranslation = estimate.targetTranslation;
            ++succeeded;
        }
    }
    m_dataset.results.clear();
    m_dataset.targetPosesReady = succeeded == processed && processed == m_dataset.samples.size();
    emitDatasetChanged();
    emit imageProcessingFinished(processed, succeeded);
    emit logMessage(QStringLiteral("标定板图片处理完成：%1/%2 成功。").arg(succeeded).arg(processed));
    if (!m_dataset.targetPosesReady)
        emit error(QStringLiteral("图片处理未完成"), QStringLiteral("存在未成功生成 target→camera 位姿的样本，请检查图片和内参。"));
    return m_dataset.targetPosesReady;
}

bool CalibrationController::ensureTargetPosesReady()
{
    return m_dataset.targetPosesReady || processBoardImages();
}

void CalibrationController::importValidationCsv(const QString &path)
{
    CalibrationDataset validationData;
    const IoResult result = readCsv(path, &validationData, m_dataset.inputSpec);
    if (!result.success) {
        emit error(QStringLiteral("导入失败"), result.error);
        return;
    }
    m_dataset.validationSamples = validationData.samples;
    m_dataset.results.clear();
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已导入独立验证 CSV：%1").arg(path));
}

void CalibrationController::importJson(const QString &path)
{
    const IoResult result = readJson(path, &m_dataset);
    if (!result.success) {
        emit error(QStringLiteral("导入失败"), result.error);
        return;
    }
    emit inputSpecChanged(m_dataset.robotName, m_dataset.cameraName);
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已导入 JSON：%1").arg(path));
}

void CalibrationController::exportCsv(const QString &path)
{
    const IoResult result = writeCsv(path, m_dataset);
    if (!result.success) emit error(QStringLiteral("导出失败"), result.error);
    else emit logMessage(QStringLiteral("已导出训练 CSV：%1（canonical Rodrigues/rad/m）").arg(path));
}

void CalibrationController::exportJson(const QString &path)
{
    const IoResult result = writeJson(path, m_dataset);
    if (!result.success) emit error(QStringLiteral("导出失败"), result.error);
    else emit logMessage(QStringLiteral("已导出 JSON：%1").arg(path));
}

void CalibrationController::exportYaml(const QString &path)
{
    const IoResult result = writeYaml(path, m_dataset);
    if (!result.success) emit error(QStringLiteral("导出失败"), result.error);
    else emit logMessage(QStringLiteral("已导出 YAML：%1").arg(path));
}

void CalibrationController::exportTxt(const QString &path, const CalibrationResult &result)
{
    const IoResult ioResult = writeResultTxt(path, m_dataset, result);
    if (!ioResult.success) emit error(QStringLiteral("导出失败"), ioResult.error);
    else emit logMessage(QStringLiteral("已导出 TXT 矩阵：%1").arg(path));
}

void CalibrationController::exportCpp(const QString &path, const CalibrationResult &result)
{
    const IoResult ioResult = writeResultCpp(path, m_dataset, result);
    if (!ioResult.success) emit error(QStringLiteral("导出失败"), ioResult.error);
    else emit logMessage(QStringLiteral("已导出 C++ 矩阵：%1").arg(path));
}

void CalibrationController::exportPython(const QString &path, const CalibrationResult &result)
{
    const IoResult ioResult = writeResultPython(path, m_dataset, result);
    if (!ioResult.success) emit error(QStringLiteral("导出失败"), ioResult.error);
    else emit logMessage(QStringLiteral("已导出 Python 矩阵：%1").arg(path));
}

void CalibrationController::deleteSamples(const QVector<int> &ids)
{
    if (ids.isEmpty()) return;
    if (m_dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
        QVector<PointSample> kept;
        for (const PointSample &sample : std::as_const(m_dataset.pointSamples))
            if (!ids.contains(sample.id)) kept.append(sample);
        m_dataset.pointSamples = kept;
        m_dataset.results.clear();
        emitDatasetChanged();
        emit logMessage(QStringLiteral("已删除 %1 组点基样本，结果已清空。").arg(ids.size()));
        return;
    }
    QVector<PoseSample> kept;
    for (const PoseSample &sample : std::as_const(m_dataset.samples))
        if (!ids.contains(sample.id)) kept.append(sample);
    m_dataset.samples = kept;
    m_dataset.results.clear();
    m_dataset.hasGroundTruth = false;
    emitDatasetChanged();
    emit logMessage(QStringLiteral("已删除 %1 组样本，结果已清空。合成真值标记已失效。").arg(ids.size()));
}

bool CalibrationController::applyManualPoseInputs(const QVector<ManualPoseInput> &inputs,
                                                   const PoseInputSpec &spec)
{
    if (inputs.size() < 3) {
        emit error(QStringLiteral("手动数据不足"), QStringLiteral("至少需要 3 组 TCP 与相机位姿。"));
        return false;
    }

    QSet<int> ids;
    QVector<PoseSample> samples;
    samples.reserve(inputs.size());
    QStringList errors;
    const PoseInputSpec tcpSpec = [&] {
        PoseInputSpec value = spec;
        value.direction = PoseDirection::GripperToBase;
        return value;
    }();
    const PoseInputSpec cameraSpec = [&] {
        PoseInputSpec value = spec;
        value.adapter = PoseAdapterKind::Generic;
        value.direction = PoseDirection::TargetToCamera;
        return value;
    }();

    for (int index = 0; index < inputs.size(); ++index) {
        const ManualPoseInput &input = inputs.at(index);
        if (input.id <= 0) {
            errors << QStringLiteral("第 %1 组样本 ID 必须为正整数。" ).arg(index + 1);
            continue;
        }
        if (ids.contains(input.id)) {
            errors << QStringLiteral("样本 ID 重复：%1。" ).arg(input.id);
            continue;
        }
        ids.insert(input.id);
        if (!matrix::isFinite(input.tcpRotation) || !matrix::isFinite(input.tcpTranslation)
            || !matrix::isFinite(input.cameraRotation) || !matrix::isFinite(input.cameraTranslation)) {
            errors << QStringLiteral("第 %1 组样本包含非有限数值。" ).arg(index + 1);
            continue;
        }
        if (spec.rotationFormat == RotationFormat::QuaternionWXYZ) {
            const double tcpNorm = input.tcpRotation[0] * input.tcpRotation[0]
                                   + input.tcpRotation[1] * input.tcpRotation[1]
                                   + input.tcpRotation[2] * input.tcpRotation[2]
                                   + input.tcpRotation[3] * input.tcpRotation[3];
            const double cameraNorm = input.cameraRotation[0] * input.cameraRotation[0]
                                      + input.cameraRotation[1] * input.cameraRotation[1]
                                      + input.cameraRotation[2] * input.cameraRotation[2]
                                      + input.cameraRotation[3] * input.cameraRotation[3];
            if (tcpNorm < 1e-24 || cameraNorm < 1e-24) {
                errors << QStringLiteral("第 %1 组样本包含零四元数。" ).arg(index + 1);
                continue;
            }
        }

        const auto tcp = pose::normalize(input.tcpRotation, input.tcpTranslation, tcpSpec);
        const auto camera = pose::normalize(input.cameraRotation, input.cameraTranslation, cameraSpec);
        if (!tcp.success || !camera.success) {
            errors << QStringLiteral("第 %1 组样本标准化失败：%2%3")
                          .arg(index + 1)
                          .arg(tcp.success ? QString{} : tcp.error)
                          .arg(camera.success ? QString{} : camera.error);
            continue;
        }

        PoseSample sample;
        sample.id = input.id;
        sample.gripperRotation = tcp.rotation;
        sample.gripperTranslation = tcp.translation;
        sample.targetRotation = camera.rotation;
        sample.targetTranslation = camera.translation;
        sample.label = input.label.trimmed().isEmpty()
                           ? QStringLiteral("手动输入 #%1").arg(input.id)
                           : input.label.trimmed();
        sample.imageStatus = ImageSampleStatus::ManualPose;
        sample.imageMessage = QStringLiteral("用户手动输入 target→camera 位姿。");
        samples.append(sample);
    }

    if (!errors.isEmpty()) {
        emit error(QStringLiteral("手动数据校验失败"), errors.join('\n'));
        return false;
    }

    CalibrationDataset candidate = m_dataset;
    candidate.mode = CalibrationMode::EyeInHand;
    candidate.inputMode = CalibrationInputMode::PosePairs;
    candidate.inputSpec = spec;
    candidate.inputSpec.direction = PoseDirection::GripperToBase;
    candidate.samples = samples;
    candidate.pointSamples.clear();
    candidate.targetPosesReady = true;
    candidate.results.clear();
    candidate.hasGroundTruth = false;
    const ValidationReport validation = validateDataset(candidate);
    if (!validation.valid) {
        emit error(QStringLiteral("手动数据不可用于标定"), validation.errors.join('\n'));
        return false;
    }

    m_dataset.inputSpec = candidate.inputSpec;
    m_dataset.mode = CalibrationMode::EyeInHand;
    m_dataset.inputMode = CalibrationInputMode::PosePairs;
    m_dataset.samples = samples;
    m_dataset.pointSamples.clear();
    m_dataset.targetPosesReady = true;
    m_dataset.results.clear();
    m_dataset.hasGroundTruth = false;
    emitDatasetChanged();
    emit reliabilityChanged(CalibrationResult{});
    emit matrixChanged(CalibrationResult{});
    emit statusChanged(QStringLiteral("已应用 %1 组手动 TCP/相机位姿，可直接执行五种算法。")
                           .arg(samples.size()));
    emit logMessage(QStringLiteral("已用手动输入位姿替换当前训练数据，共 %1 组。")
                        .arg(samples.size()));
    return true;
}

bool CalibrationController::applyManualPointInputs(const QVector<PointSample> &inputs,
                                                    const PoseInputSpec &spec)
{
    if (inputs.size() < 5) {
        emit error(QStringLiteral("点基数据不足"),
                   QStringLiteral("FixedPoint3D 至少需要 5 组 TCP 与相机 XYZ 数据。"));
        return false;
    }

    QSet<int> ids;
    QVector<PointSample> samples;
    samples.reserve(inputs.size());
    QStringList errors;
    PoseInputSpec tcpSpec = spec;
    tcpSpec.direction = PoseDirection::GripperToBase;
    const double lengthScale = spec.lengthUnit == LengthUnit::Millimeters ? 1e-3 : 1.0;
    for (int index = 0; index < inputs.size(); ++index) {
        const PointSample &input = inputs.at(index);
        if (input.id <= 0) {
            errors << QStringLiteral("第 %1 组样本 ID 必须为正整数。").arg(index + 1);
            continue;
        }
        if (ids.contains(input.id)) {
            errors << QStringLiteral("样本 ID 重复：%1。").arg(input.id);
            continue;
        }
        ids.insert(input.id);
        if (!matrix::isFinite(input.gripperRotation) || !matrix::isFinite(input.gripperTranslation)
            || !matrix::isFinite(input.cameraPoint)) {
            errors << QStringLiteral("第 %1 组包含非有限数值。").arg(index + 1);
            continue;
        }
        if (spec.rotationFormat == RotationFormat::QuaternionWXYZ) {
            errors << QStringLiteral("FixedPoint3D 手动输入的 TCP 旋转需要 3 个轴值；请使用 Rodrigues、Euler XYZ 或 RPY。")
                          .arg(index + 1);
            continue;
        }

        const Vector4 rotation{input.gripperRotation[0], input.gripperRotation[1],
                               input.gripperRotation[2], 0.0};
        const auto tcp = pose::normalize(rotation, input.gripperTranslation, tcpSpec);
        if (!tcp.success) {
            errors << QStringLiteral("第 %1 组 TCP 标准化失败：%2").arg(index + 1).arg(tcp.error);
            continue;
        }
        PointSample sample;
        sample.id = input.id;
        sample.gripperRotation = tcp.rotation;
        sample.gripperTranslation = tcp.translation;
        sample.cameraPoint = {input.cameraPoint[0] * lengthScale,
                              input.cameraPoint[1] * lengthScale,
                              input.cameraPoint[2] * lengthScale};
        sample.label = input.label.trimmed().isEmpty()
                           ? QStringLiteral("手动点基输入 #%1").arg(input.id)
                           : input.label.trimmed();
        samples.append(sample);
    }

    if (!errors.isEmpty()) {
        emit error(QStringLiteral("FixedPoint3D 数据校验失败"), errors.join('\n'));
        return false;
    }

    m_dataset.mode = CalibrationMode::EyeInHand;
    m_dataset.inputMode = CalibrationInputMode::FixedPoint3D;
    m_dataset.inputSpec = spec;
    m_dataset.inputSpec.direction = PoseDirection::GripperToBase;
    m_dataset.pointSamples = samples;
    m_dataset.samples.clear();
    m_dataset.targetPosesReady = false;
    m_dataset.results.clear();
    m_dataset.hasGroundTruth = false;
    emitDatasetChanged();
    emit reliabilityChanged(CalibrationResult{});
    emit matrixChanged(CalibrationResult{});
    emit statusChanged(QStringLiteral("已应用 %1 组 FixedPoint3D 点基数据。相机端仅使用 XYZ，不需要 rx/ry/rz。")
                           .arg(samples.size()));
    emit logMessage(QStringLiteral("手动点基数据已替换当前训练集，共 %1 组。").arg(samples.size()));
    return true;
}

FixedTargetPoseReport CalibrationController::computeFixedTargetPose(const CalibrationResult &result,
                                                                     int referenceSampleId)
{
    if (m_dataset.inputMode != CalibrationInputMode::PosePairs || !result.success) {
        FixedTargetPoseReport report;
        report.errors << QStringLiteral("当前需要成功的 PosePairs 结果才能计算 fixed target pose。");
        return report;
    }
    const FixedTargetPoseReport report = PoseQualityService::computeFixedTargetPose(
        m_dataset, result.cameraToGripper, referenceSampleId);
    CalibrationResult updated = result;
    for (CalibrationResult &stored : m_dataset.results) {
        if (stored.method == result.method) {
            stored.fixedTargetReport = report;
            stored.qualityReport = PoseQualityService::evaluatePoseQuality(m_dataset);
            updated = stored;
        }
    }
    emitDatasetChanged();
    emit reliabilityChanged(updated);
    return report;
}

CalibrationResult CalibrationController::optimizeRecommendedResult()
{
    CalibrationResult optimized;
    const CalibrationResult seed = recommendedResult();
    if (m_dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
        optimized = PointCalibrationService::calibrate(m_dataset);
    } else {
        if (!seed.success) {
            emit error(QStringLiteral("无法精修"), QStringLiteral("请先计算一个成功的推荐结果。"));
            return optimized;
        }
        optimized = NonlinearOptimizer::refinePose(m_dataset, seed);
    }
    for (CalibrationResult &stored : m_dataset.results) stored.recommended = false;
    if (optimized.success) {
        optimized.recommended = true;
        bool replaced = false;
        for (CalibrationResult &stored : m_dataset.results) {
            if (stored.method == optimized.method) {
                stored = optimized;
                replaced = true;
                break;
            }
        }
        if (!replaced) m_dataset.results.append(optimized);
    }
    if (optimized.method == CalibrationMethod::PointBased && optimized.success) {
        m_dataset.results = {optimized};
    }
    if (optimized.success) applyResiduals(optimized.trainingReport);
    emitDatasetChanged();
    emit reliabilityChanged(optimized);
    emit matrixChanged(optimized);
    emit logMessage(optimized.message);
    return optimized;
}

PoseQualityReport CalibrationController::evaluatePoseQuality() const
{
    return m_dataset.inputMode == CalibrationInputMode::FixedPoint3D
               ? PoseQualityService::evaluatePointQuality(m_dataset)
               : PoseQualityService::evaluatePoseQuality(m_dataset);
}

void CalibrationController::calculateSelected(CalibrationMethod method)
{
    m_dataset.mode = CalibrationMode::EyeInHand;
    if (m_dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
        if (m_dataset.pointSamples.size() < 5) {
            emit error(QStringLiteral("无法计算"), QStringLiteral("FixedPoint3D 至少需要 5 组点基样本。"));
            return;
        }
        emit calculationStarted();
        const CalibrationDataset dataset = m_dataset;
        auto *watcher = new QFutureWatcher<CalibrationResult>(this);
        connect(watcher, &QFutureWatcher<CalibrationResult>::finished, this, [this, watcher]() {
            const CalibrationResult result = watcher->result();
            m_dataset.results = result.success ? QVector<CalibrationResult>{result} : QVector<CalibrationResult>{};
            if (result.success) emit reliabilityChanged(result);
            emitDatasetChanged();
            if (result.success) emit matrixChanged(result);
            emit logMessage(result.message);
            emit calculationFinished();
            watcher->deleteLater();
        });
        watcher->setFuture(QtConcurrent::run([dataset]() {
            return PointCalibrationService::calibrate(dataset);
        }));
        Q_UNUSED(method)
        return;
    }
    if (!ensureTargetPosesReady()) return;
    const ValidationReport validation = validateDataset(m_dataset);
    if (!validation.valid) {
        emit error(QStringLiteral("无法计算"), validation.errors.join('\n'));
        return;
    }

    emit calculationStarted();

    const CalibrationDataset dataset = m_dataset;
    auto *watcher = new QFutureWatcher<CalibrationResult>(this);
    connect(watcher, &QFutureWatcher<CalibrationResult>::finished, this, [this, watcher, method]() {
        const CalibrationResult result = watcher->result();
        m_dataset.results = {result};
        applyResiduals(result.trainingReport);
        emitDatasetChanged();
        emit reliabilityChanged(result);
        emit matrixChanged(result);
        emit logMessage(QStringLiteral("%1：%2（耗时 %3 ms）")
                            .arg(methodName(method), result.message).arg(result.elapsedMs));
        emit calculationFinished();
        watcher->deleteLater();
    });
    watcher->setFuture(QtConcurrent::run([dataset, method]() {
        return CalibrationService::calibrate(dataset, method);
    }));
}

void CalibrationController::calculateAll()
{
    if (m_dataset.inputMode == CalibrationInputMode::FixedPoint3D) {
        calculateSelected(CalibrationMethod::PointBased);
        return;
    }
    m_dataset.mode = CalibrationMode::EyeInHand;
    if (!ensureTargetPosesReady()) return;
    const ValidationReport validation = validateDataset(m_dataset);
    if (!validation.valid) {
        emit error(QStringLiteral("无法计算"), validation.errors.join('\n'));
        return;
    }

    emit calculationStarted();

    const CalibrationDataset dataset = m_dataset;
    auto *watcher = new QFutureWatcher<QVector<CalibrationResult>>(this);
    connect(watcher, &QFutureWatcher<QVector<CalibrationResult>>::finished, this, [this, watcher]() {
        m_dataset.results = watcher->result();
        if (!m_dataset.results.isEmpty())
            applyResiduals(recommendedResult().trainingReport);
        emitDatasetChanged();

        int recommendedRow = -1;
        for (int index = 0; index < m_dataset.results.size(); ++index) {
            const CalibrationResult &result = m_dataset.results.at(index);
            emit logMessage(QStringLiteral("%1：%2，RMSE %3° / %4 m，%5")
                                .arg(methodName(result.method), result.message)
                                .arg(result.trainingReport.rotationRmseDeg, 0, 'f', 5)
                                .arg(result.trainingReport.translationRmseM, 0, 'f', 7)
                                .arg(result.trainingReport.passed ? QStringLiteral("通过")
                                                                  : QStringLiteral("未通过")));
            if (result.recommended) recommendedRow = index;
        }

        const CalibrationResult result = recommendedResult();
        emit reliabilityChanged(result);
        if (!m_dataset.results.isEmpty()) emit matrixChanged(result);
        emit calculationFinished();
        watcher->deleteLater();
    });
    watcher->setFuture(QtConcurrent::run([dataset]() {
        return CalibrationService::calibrateAll(dataset);
    }));
}

} // namespace handeye
