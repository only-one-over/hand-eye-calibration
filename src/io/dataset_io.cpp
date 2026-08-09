#include "io/dataset_io.h"

#include "core/pose_conversion.h"
#include "io/pose_adapter.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QTextStream>

namespace handeye {

namespace {

QJsonArray vectorToJson(const Vector3 &value)
{
    return {value[0], value[1], value[2]};
}

QJsonArray vector2ToJson(const Vector2 &value)
{
    return {value[0], value[1]};
}

Vector2 vector2FromJson(const QJsonValue &value)
{
    const QJsonArray array = value.toArray();
    if (array.size() < 2) return {};
    return {array.at(0).toDouble(), array.at(1).toDouble()};
}

Vector3 vectorFromJson(const QJsonValue &value)
{
    const QJsonArray array = value.toArray();
    if (array.size() < 3) return {};
    return {array.at(0).toDouble(), array.at(1).toDouble(), array.at(2).toDouble()};
}

QJsonArray matrixToJson(const Matrix4 &matrix)
{
    QJsonArray rows;
    for (const auto &row : matrix) {
        QJsonArray values;
        for (double value : row) values.append(value);
        rows.append(values);
    }
    return rows;
}

QJsonArray matrix3ToJson(const Matrix3 &matrix)
{
    QJsonArray rows;
    for (const auto &row : matrix) rows.append(QJsonArray{row[0], row[1], row[2]});
    return rows;
}

QJsonArray vector5ToJson(const Vector5 &value)
{
    QJsonArray array;
    for (double item : value) array.append(item);
    return array;
}

Vector5 vector5FromJson(const QJsonValue &value)
{
    Vector5 result{};
    const QJsonArray array = value.toArray();
    for (int index = 0; index < 5 && index < array.size(); ++index)
        result[index] = array.at(index).toDouble();
    return result;
}

Matrix3 matrix3FromJson(const QJsonValue &value)
{
    Matrix3 matrix{};
    const QJsonArray rows = value.toArray();
    for (int row = 0; row < 3 && row < rows.size(); ++row) {
        const QJsonArray values = rows.at(row).toArray();
        for (int col = 0; col < 3 && col < values.size(); ++col)
            matrix[row][col] = values.at(col).toDouble();
    }
    return matrix;
}

Matrix4 matrixFromJson(const QJsonValue &value)
{
    Matrix4 matrix{};
    const QJsonArray rows = value.toArray();
    for (int row = 0; row < 4 && row < rows.size(); ++row) {
        const QJsonArray values = rows.at(row).toArray();
        for (int col = 0; col < 4 && col < values.size(); ++col)
            matrix[row][col] = values.at(col).toDouble();
    }
    return matrix;
}

QString modeToString(CalibrationMode mode)
{
    return mode == CalibrationMode::EyeToHand ? QStringLiteral("eye_to_hand")
                                               : QStringLiteral("eye_in_hand");
}

QString inputModeToString(CalibrationInputMode mode)
{
    return mode == CalibrationInputMode::FixedPoint3D ? QStringLiteral("fixed_point_3d")
                                                      : QStringLiteral("pose_pairs");
}

CalibrationInputMode inputModeFromString(const QString &value)
{
    return value == QStringLiteral("fixed_point_3d") ? CalibrationInputMode::FixedPoint3D
                                                       : CalibrationInputMode::PosePairs;
}

QJsonObject pointSampleToJson(const PointSample &sample)
{
    return {{QStringLiteral("id"), sample.id},
            {QStringLiteral("label"), sample.label},
            {QStringLiteral("gripper_rotation_rodrigues_rad"), vectorToJson(sample.gripperRotation)},
            {QStringLiteral("gripper_translation_m"), vectorToJson(sample.gripperTranslation)},
            {QStringLiteral("camera_point_m"), vectorToJson(sample.cameraPoint)},
            {QStringLiteral("predicted_base_point_m"), vectorToJson(sample.predictedBasePoint)},
            {QStringLiteral("residual_m"), sample.residualM},
            {QStringLiteral("outlier"), sample.outlier}};
}

PointSample pointSampleFromJson(const QJsonObject &object)
{
    PointSample sample;
    sample.id = object.value(QStringLiteral("id")).toInt();
    sample.label = object.value(QStringLiteral("label")).toString();
    sample.gripperRotation = vectorFromJson(object.value(QStringLiteral("gripper_rotation_rodrigues_rad")));
    sample.gripperTranslation = vectorFromJson(object.value(QStringLiteral("gripper_translation_m")));
    sample.cameraPoint = vectorFromJson(object.value(QStringLiteral("camera_point_m")));
    sample.predictedBasePoint = vectorFromJson(object.value(QStringLiteral("predicted_base_point_m")));
    sample.residualM = object.value(QStringLiteral("residual_m")).toDouble();
    sample.outlier = object.value(QStringLiteral("outlier")).toBool();
    return sample;
}

CalibrationMode modeFromString(const QString &value)
{
    return value == QStringLiteral("eye_to_hand") ? CalibrationMode::EyeToHand
                                                   : CalibrationMode::EyeInHand;
}

QString formatToString(RotationFormat format) { return rotationFormatName(format); }
QString adapterToString(PoseAdapterKind adapter) { return adapterName(adapter); }

BoardPattern boardPatternFromString(const QString &value)
{
    if (value == QStringLiteral("ChArUco")) return BoardPattern::Charuco;
    if (value == QStringLiteral("ArUco Grid")) return BoardPattern::ArucoGrid;
    return BoardPattern::Chessboard;
}

ChessboardDetector detectorFromString(const QString &value)
{
    if (value == QStringLiteral("Classic")) return ChessboardDetector::Classic;
    if (value == QStringLiteral("SB")) return ChessboardDetector::SB;
    return ChessboardDetector::Auto;
}

PnpMethod pnpFromString(const QString &value)
{
    if (value == QStringLiteral("ITERATIVE")) return PnpMethod::Iterative;
    if (value == QStringLiteral("IPPE")) return PnpMethod::IPPE;
    return PnpMethod::Auto;
}

RotationFormat formatFromString(const QString &value)
{
    if (value == QStringLiteral("Euler XYZ")) return RotationFormat::EulerXYZ;
    if (value == QStringLiteral("RPY (Z-Y-X)")) return RotationFormat::RPY;
    if (value == QStringLiteral("Quaternion (w,x,y,z)")) return RotationFormat::QuaternionWXYZ;
    return RotationFormat::Rodrigues;
}

PoseAdapterKind adapterFromString(const QString &value)
{
    if (value.startsWith(QStringLiteral("Universal Robots"))) return PoseAdapterKind::UniversalRobots;
    if (value.startsWith(QStringLiteral("KUKA"))) return PoseAdapterKind::Kuka;
    if (value.startsWith(QStringLiteral("FANUC"))) return PoseAdapterKind::Fanuc;
    return PoseAdapterKind::Generic;
}

QJsonObject sampleToJson(const PoseSample &sample)
{
    return {{QStringLiteral("id"), sample.id},
            {QStringLiteral("label"), sample.label},
            {QStringLiteral("gripper_rotation_rodrigues_rad"), vectorToJson(sample.gripperRotation)},
            {QStringLiteral("gripper_translation_m"), vectorToJson(sample.gripperTranslation)},
            {QStringLiteral("target_rotation_rodrigues_rad"), vectorToJson(sample.targetRotation)},
            {QStringLiteral("target_translation_m"), vectorToJson(sample.targetTranslation)},
            {QStringLiteral("rotation_residual_deg"), sample.rotationResidualDeg},
            {QStringLiteral("translation_residual_m"), sample.translationResidualM},
            {QStringLiteral("outlier"), sample.outlier},
            {QStringLiteral("image_path"), sample.imagePath},
            {QStringLiteral("image_status"), imageSampleStatusName(sample.imageStatus)},
            {QStringLiteral("detected_corner_count"), sample.detectedCornerCount},
            {QStringLiteral("pnp_reprojection_rmse_px"), sample.pnpReprojectionRmsePx},
            {QStringLiteral("image_message"), sample.imageMessage},
            {QStringLiteral("image_width"), sample.imageWidth},
            {QStringLiteral("image_height"), sample.imageHeight},
            {QStringLiteral("image_center_x_norm"), sample.imageCenterXNorm},
            {QStringLiteral("image_center_y_norm"), sample.imageCenterYNorm},
            {QStringLiteral("detection_method"), sample.detectionMethod},
            {QStringLiteral("selected_pnp_method"), pnpMethodName(sample.selectedPnpMethod)},
            {QStringLiteral("iterative_pnp_rmse_px"), sample.iterativePnpRmsePx},
            {QStringLiteral("ippe_pnp_rmse_px"), sample.ippePnpRmsePx}};
}

PoseSample sampleFromJson(const QJsonObject &object)
{
    PoseSample sample;
    sample.id = object.value(QStringLiteral("id")).toInt();
    sample.label = object.value(QStringLiteral("label")).toString();
    sample.gripperRotation = vectorFromJson(object.value(QStringLiteral("gripper_rotation_rodrigues_rad")));
    sample.gripperTranslation = vectorFromJson(object.value(QStringLiteral("gripper_translation_m")));
    sample.targetRotation = vectorFromJson(object.value(QStringLiteral("target_rotation_rodrigues_rad")));
    sample.targetTranslation = vectorFromJson(object.value(QStringLiteral("target_translation_m")));
    sample.rotationResidualDeg = object.value(QStringLiteral("rotation_residual_deg")).toDouble();
    sample.translationResidualM = object.value(QStringLiteral("translation_residual_m")).toDouble();
    sample.outlier = object.value(QStringLiteral("outlier")).toBool();
    sample.imagePath = object.value(QStringLiteral("image_path")).toString();
    sample.detectedCornerCount = object.value(QStringLiteral("detected_corner_count")).toInt();
    sample.pnpReprojectionRmsePx = object.value(QStringLiteral("pnp_reprojection_rmse_px")).toDouble();
    sample.imageMessage = object.value(QStringLiteral("image_message")).toString();
    sample.imageWidth = object.value(QStringLiteral("image_width")).toInt();
    sample.imageHeight = object.value(QStringLiteral("image_height")).toInt();
    sample.imageCenterXNorm = object.value(QStringLiteral("image_center_x_norm")).toDouble(0.5);
    sample.imageCenterYNorm = object.value(QStringLiteral("image_center_y_norm")).toDouble(0.5);
    sample.detectionMethod = object.value(QStringLiteral("detection_method")).toString();
    sample.selectedPnpMethod = pnpFromString(object.value(QStringLiteral("selected_pnp_method")).toString());
    sample.iterativePnpRmsePx = object.value(QStringLiteral("iterative_pnp_rmse_px")).toDouble();
    sample.ippePnpRmsePx = object.value(QStringLiteral("ippe_pnp_rmse_px")).toDouble();
    sample.imageStatus = imageSampleStatusFromName(object.value(QStringLiteral("image_status")).toString());
    return sample;
}

QJsonObject reportToJson(const ReliabilityReport &report)
{
    QJsonArray residuals;
    for (const SampleResidual &residual : report.sampleResiduals) {
        residuals.append(QJsonObject{{QStringLiteral("sample_id"), residual.sampleId},
                                     {QStringLiteral("rotation_error_deg"), residual.rotationErrorDeg},
                                     {QStringLiteral("translation_error_m"), residual.translationErrorM},
                                     {QStringLiteral("normalized_score"), residual.normalizedScore},
                                     {QStringLiteral("pair_count"), residual.pairCount},
                                     {QStringLiteral("outlier"), residual.outlier}});
    }
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("valid"), report.valid},
            {QStringLiteral("passed"), report.passed}, {QStringLiteral("sample_count"), report.sampleCount},
            {QStringLiteral("outlier_count"), report.outlierCount},
            {QStringLiteral("rotation_rmse_deg"), report.rotationRmseDeg},
            {QStringLiteral("translation_rmse_m"), report.translationRmseM},
            {QStringLiteral("rotation_mean_deg"), report.rotationMeanDeg},
            {QStringLiteral("translation_mean_m"), report.translationMeanM},
            {QStringLiteral("rotation_max_deg"), report.rotationMaxDeg},
            {QStringLiteral("translation_max_m"), report.translationMaxM},
            {QStringLiteral("errors"), QJsonArray::fromStringList(report.errors)},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)},
            {QStringLiteral("sample_residuals"), residuals}};
}

ReliabilityReport reportFromJson(const QJsonObject &object)
{
    ReliabilityReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.valid = object.value(QStringLiteral("valid")).toBool();
    report.passed = object.value(QStringLiteral("passed")).toBool();
    report.sampleCount = object.value(QStringLiteral("sample_count")).toInt();
    report.outlierCount = object.value(QStringLiteral("outlier_count")).toInt();
    report.rotationRmseDeg = object.value(QStringLiteral("rotation_rmse_deg")).toDouble();
    report.translationRmseM = object.value(QStringLiteral("translation_rmse_m")).toDouble();
    report.rotationMeanDeg = object.value(QStringLiteral("rotation_mean_deg")).toDouble();
    report.translationMeanM = object.value(QStringLiteral("translation_mean_m")).toDouble();
    report.rotationMaxDeg = object.value(QStringLiteral("rotation_max_deg")).toDouble();
    report.translationMaxM = object.value(QStringLiteral("translation_max_m")).toDouble();
    for (const QJsonValue &value : object.value(QStringLiteral("errors")).toArray()) report.errors.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray()) report.warnings.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("sample_residuals")).toArray()) {
        const QJsonObject item = value.toObject();
        SampleResidual residual;
        residual.sampleId = item.value(QStringLiteral("sample_id")).toInt();
        residual.rotationErrorDeg = item.value(QStringLiteral("rotation_error_deg")).toDouble();
        residual.translationErrorM = item.value(QStringLiteral("translation_error_m")).toDouble();
        residual.normalizedScore = item.value(QStringLiteral("normalized_score")).toDouble();
        residual.pairCount = item.value(QStringLiteral("pair_count")).toInt();
        residual.outlier = item.value(QStringLiteral("outlier")).toBool();
        report.sampleResiduals.append(residual);
    }
    return report;
}

FixedTargetPoseReport fixedTargetReportFromJson(const QJsonObject &object)
{
    FixedTargetPoseReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.success = object.value(QStringLiteral("success")).toBool();
    report.referenceSampleId = object.value(QStringLiteral("reference_sample_id")).toInt(-1);
    report.robustMeanPose = matrixFromJson(object.value(QStringLiteral("robust_mean_pose")));
    report.robustMeanRotation = vectorFromJson(object.value(QStringLiteral("robust_mean_rotation")));
    report.robustMeanTranslation = vectorFromJson(object.value(QStringLiteral("robust_mean_translation")));
    report.rotationRmseDeg = object.value(QStringLiteral("rotation_rmse_deg")).toDouble();
    report.translationRmseM = object.value(QStringLiteral("translation_rmse_m")).toDouble();
    report.rotationMeanDeg = object.value(QStringLiteral("rotation_mean_deg")).toDouble();
    report.translationMeanM = object.value(QStringLiteral("translation_mean_m")).toDouble();
    report.rotationMaxDeg = object.value(QStringLiteral("rotation_max_deg")).toDouble();
    report.translationMaxM = object.value(QStringLiteral("translation_max_m")).toDouble();
    report.outlierCount = object.value(QStringLiteral("outlier_count")).toInt();
    for (const QJsonValue &value : object.value(QStringLiteral("errors")).toArray()) report.errors.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray()) report.warnings.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("samples")).toArray()) {
        const QJsonObject item = value.toObject();
        FixedTargetPoseSample sample;
        sample.sampleId = item.value(QStringLiteral("sample_id")).toInt();
        sample.predictedPose = matrixFromJson(item.value(QStringLiteral("predicted_pose")));
        sample.predictedRotation = vectorFromJson(item.value(QStringLiteral("predicted_rotation")));
        sample.predictedTranslation = vectorFromJson(item.value(QStringLiteral("predicted_translation")));
        sample.rotationErrorToMeanDeg = item.value(QStringLiteral("rotation_error_to_mean_deg")).toDouble();
        sample.translationErrorToMeanM = item.value(QStringLiteral("translation_error_to_mean_m")).toDouble();
        sample.rotationErrorToReferenceDeg = item.value(QStringLiteral("rotation_error_to_reference_deg")).toDouble();
        sample.translationErrorToReferenceM = item.value(QStringLiteral("translation_error_to_reference_m")).toDouble();
        sample.outlier = item.value(QStringLiteral("outlier")).toBool();
        report.samples.append(sample);
    }
    return report;
}

FixedPointReport fixedPointReportFromJson(const QJsonObject &object)
{
    FixedPointReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.success = object.value(QStringLiteral("success")).toBool();
    report.robustMeanPoint = vectorFromJson(object.value(QStringLiteral("robust_mean_point_m")));
    report.rmseM = object.value(QStringLiteral("rmse_m")).toDouble();
    report.meanErrorM = object.value(QStringLiteral("mean_error_m")).toDouble();
    report.maxErrorM = object.value(QStringLiteral("max_error_m")).toDouble();
    report.outlierCount = object.value(QStringLiteral("outlier_count")).toInt();
    for (const QJsonValue &value : object.value(QStringLiteral("errors")).toArray()) report.errors.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray()) report.warnings.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("samples")).toArray()) {
        const QJsonObject item = value.toObject();
        FixedPointSample sample;
        sample.sampleId = item.value(QStringLiteral("sample_id")).toInt();
        sample.predictedBasePoint = vectorFromJson(item.value(QStringLiteral("predicted_base_point_m")));
        sample.residualM = item.value(QStringLiteral("residual_m")).toDouble();
        sample.outlier = item.value(QStringLiteral("outlier")).toBool();
        report.samples.append(sample);
    }
    return report;
}

PoseQualityReport qualityReportFromJson(const QJsonObject &object)
{
    PoseQualityReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.calculable = object.value(QStringLiteral("calculable")).toBool();
    report.sampleScore = object.value(QStringLiteral("sample_score")).toInt();
    report.rotationAmplitudeScore = object.value(QStringLiteral("rotation_amplitude_score")).toInt();
    report.rotationAxisScore = object.value(QStringLiteral("rotation_axis_score")).toInt();
    report.spatialDistributionScore = object.value(QStringLiteral("spatial_distribution_score")).toInt();
    report.totalScore = object.value(QStringLiteral("total_score")).toInt();
    report.level = object.value(QStringLiteral("level")).toString();
    report.maxRelativeRotationDeg = object.value(QStringLiteral("max_relative_rotation_deg")).toDouble();
    report.independentAxisCount = object.value(QStringLiteral("independent_axis_count")).toInt();
    report.nearMidFarCoverage = object.value(QStringLiteral("near_mid_far_coverage")).toBool();
    report.fullFovCoverage = object.value(QStringLiteral("full_fov_coverage")).toBool();
    report.imageCoverageAvailable = object.value(QStringLiteral("image_coverage_available")).toBool();
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray()) report.warnings.append(value.toString());
    return report;
}

NonlinearOptimizationReport optimizationReportFromJson(const QJsonObject &object)
{
    NonlinearOptimizationReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.success = object.value(QStringLiteral("success")).toBool();
    report.converged = object.value(QStringLiteral("converged")).toBool();
    report.iterations = object.value(QStringLiteral("iterations")).toInt();
    report.huberOutlierCount = object.value(QStringLiteral("huber_outlier_count")).toInt();
    report.beforeRotationRmseDeg = object.value(QStringLiteral("before_rotation_rmse_deg")).toDouble();
    report.beforeTranslationRmseM = object.value(QStringLiteral("before_translation_rmse_m")).toDouble();
    report.afterRotationRmseDeg = object.value(QStringLiteral("after_rotation_rmse_deg")).toDouble();
    report.afterTranslationRmseM = object.value(QStringLiteral("after_translation_rmse_m")).toDouble();
    report.message = object.value(QStringLiteral("message")).toString();
    return report;
}

BootstrapReport bootstrapReportFromJson(const QJsonObject &object)
{
    BootstrapReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.success = object.value(QStringLiteral("success")).toBool();
    report.requestedResamples = object.value(QStringLiteral("requested_resamples")).toInt();
    report.successfulResamples = object.value(QStringLiteral("successful_resamples")).toInt();
    report.confidenceLevel = object.value(QStringLiteral("confidence_level")).toDouble(0.95);
    report.rotationStdDeg = vectorFromJson(object.value(QStringLiteral("rotation_std_deg")));
    report.translationStdM = vectorFromJson(object.value(QStringLiteral("translation_std_m")));
    report.rotationLowerDeg = vectorFromJson(object.value(QStringLiteral("rotation_lower_deg")));
    report.rotationUpperDeg = vectorFromJson(object.value(QStringLiteral("rotation_upper_deg")));
    report.translationLowerM = vectorFromJson(object.value(QStringLiteral("translation_lower_m")));
    report.translationUpperM = vectorFromJson(object.value(QStringLiteral("translation_upper_m")));
    report.rotationNormStdDeg = object.value(QStringLiteral("rotation_norm_std_deg")).toDouble();
    report.translationNormStdM = object.value(QStringLiteral("translation_norm_std_m")).toDouble();
    report.confidenceScore = object.value(QStringLiteral("confidence_score")).toDouble();
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray())
        report.warnings.append(value.toString());
    report.message = object.value(QStringLiteral("message")).toString();
    return report;
}

PnpQualityReport pnpQualityReportFromJson(const QJsonObject &object)
{
    PnpQualityReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.passed = object.value(QStringLiteral("passed")).toBool();
    report.totalImageSamples = object.value(QStringLiteral("total_image_samples")).toInt();
    report.validSamples = object.value(QStringLiteral("valid_samples")).toInt();
    report.outlierCount = object.value(QStringLiteral("outlier_count")).toInt();
    report.meanRmsePx = object.value(QStringLiteral("mean_rmse_px")).toDouble();
    report.maxRmsePx = object.value(QStringLiteral("max_rmse_px")).toDouble();
    report.thresholdPx = object.value(QStringLiteral("threshold_px")).toDouble(3.0);
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray())
        report.warnings.append(value.toString());
    return report;
}

QJsonObject fixedTargetReportToJson(const FixedTargetPoseReport &report)
{
    QJsonArray samples;
    for (const FixedTargetPoseSample &sample : report.samples) {
        samples.append(QJsonObject{{QStringLiteral("sample_id"), sample.sampleId},
                                   {QStringLiteral("predicted_pose"), matrixToJson(sample.predictedPose)},
                                   {QStringLiteral("predicted_rotation"), vectorToJson(sample.predictedRotation)},
                                   {QStringLiteral("predicted_translation"), vectorToJson(sample.predictedTranslation)},
                                   {QStringLiteral("rotation_error_to_mean_deg"), sample.rotationErrorToMeanDeg},
                                   {QStringLiteral("translation_error_to_mean_m"), sample.translationErrorToMeanM},
                                   {QStringLiteral("rotation_error_to_reference_deg"), sample.rotationErrorToReferenceDeg},
                                   {QStringLiteral("translation_error_to_reference_m"), sample.translationErrorToReferenceM},
                                   {QStringLiteral("outlier"), sample.outlier}});
    }
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("success"), report.success},
            {QStringLiteral("reference_sample_id"), report.referenceSampleId},
            {QStringLiteral("robust_mean_pose"), matrixToJson(report.robustMeanPose)},
            {QStringLiteral("robust_mean_rotation"), vectorToJson(report.robustMeanRotation)},
            {QStringLiteral("robust_mean_translation"), vectorToJson(report.robustMeanTranslation)},
            {QStringLiteral("rotation_rmse_deg"), report.rotationRmseDeg},
            {QStringLiteral("translation_rmse_m"), report.translationRmseM},
            {QStringLiteral("rotation_mean_deg"), report.rotationMeanDeg},
            {QStringLiteral("translation_mean_m"), report.translationMeanM},
            {QStringLiteral("rotation_max_deg"), report.rotationMaxDeg},
            {QStringLiteral("translation_max_m"), report.translationMaxM},
            {QStringLiteral("outlier_count"), report.outlierCount},
            {QStringLiteral("errors"), QJsonArray::fromStringList(report.errors)},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)},
            {QStringLiteral("samples"), samples}};
}

QJsonObject fixedPointReportToJson(const FixedPointReport &report)
{
    QJsonArray samples;
    for (const FixedPointSample &sample : report.samples)
        samples.append(QJsonObject{{QStringLiteral("sample_id"), sample.sampleId},
                                   {QStringLiteral("predicted_base_point_m"), vectorToJson(sample.predictedBasePoint)},
                                   {QStringLiteral("residual_m"), sample.residualM},
                                   {QStringLiteral("outlier"), sample.outlier}});
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("success"), report.success},
            {QStringLiteral("robust_mean_point_m"), vectorToJson(report.robustMeanPoint)},
            {QStringLiteral("rmse_m"), report.rmseM}, {QStringLiteral("mean_error_m"), report.meanErrorM},
            {QStringLiteral("max_error_m"), report.maxErrorM}, {QStringLiteral("outlier_count"), report.outlierCount},
            {QStringLiteral("errors"), QJsonArray::fromStringList(report.errors)},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)},
            {QStringLiteral("samples"), samples}};
}

QJsonObject qualityReportToJson(const PoseQualityReport &report)
{
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("calculable"), report.calculable},
            {QStringLiteral("sample_score"), report.sampleScore},
            {QStringLiteral("rotation_amplitude_score"), report.rotationAmplitudeScore},
            {QStringLiteral("rotation_axis_score"), report.rotationAxisScore},
            {QStringLiteral("spatial_distribution_score"), report.spatialDistributionScore},
            {QStringLiteral("total_score"), report.totalScore}, {QStringLiteral("level"), report.level},
            {QStringLiteral("max_relative_rotation_deg"), report.maxRelativeRotationDeg},
            {QStringLiteral("independent_axis_count"), report.independentAxisCount},
            {QStringLiteral("near_mid_far_coverage"), report.nearMidFarCoverage},
            {QStringLiteral("full_fov_coverage"), report.fullFovCoverage},
            {QStringLiteral("image_coverage_available"), report.imageCoverageAvailable},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)}};
}

QJsonObject optimizationReportToJson(const NonlinearOptimizationReport &report)
{
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("success"), report.success},
            {QStringLiteral("converged"), report.converged}, {QStringLiteral("iterations"), report.iterations},
            {QStringLiteral("huber_outlier_count"), report.huberOutlierCount},
            {QStringLiteral("before_rotation_rmse_deg"), report.beforeRotationRmseDeg},
            {QStringLiteral("before_translation_rmse_m"), report.beforeTranslationRmseM},
            {QStringLiteral("after_rotation_rmse_deg"), report.afterRotationRmseDeg},
            {QStringLiteral("after_translation_rmse_m"), report.afterTranslationRmseM},
            {QStringLiteral("message"), report.message}};
}

QJsonObject pnpQualityReportToJson(const PnpQualityReport &report)
{
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("passed"), report.passed},
            {QStringLiteral("total_image_samples"), report.totalImageSamples},
            {QStringLiteral("valid_samples"), report.validSamples},
            {QStringLiteral("outlier_count"), report.outlierCount},
            {QStringLiteral("mean_rmse_px"), report.meanRmsePx},
            {QStringLiteral("max_rmse_px"), report.maxRmsePx},
            {QStringLiteral("threshold_px"), report.thresholdPx},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)}};
}

QJsonObject bootstrapReportToJson(const BootstrapReport &report)
{
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("success"), report.success},
            {QStringLiteral("requested_resamples"), report.requestedResamples},
            {QStringLiteral("successful_resamples"), report.successfulResamples},
            {QStringLiteral("confidence_level"), report.confidenceLevel},
            {QStringLiteral("rotation_std_deg"), vectorToJson(report.rotationStdDeg)},
            {QStringLiteral("translation_std_m"), vectorToJson(report.translationStdM)},
            {QStringLiteral("rotation_lower_deg"), vectorToJson(report.rotationLowerDeg)},
            {QStringLiteral("rotation_upper_deg"), vectorToJson(report.rotationUpperDeg)},
            {QStringLiteral("translation_lower_m"), vectorToJson(report.translationLowerM)},
            {QStringLiteral("translation_upper_m"), vectorToJson(report.translationUpperM)},
            {QStringLiteral("rotation_norm_std_deg"), report.rotationNormStdDeg},
            {QStringLiteral("translation_norm_std_m"), report.translationNormStdM},
            {QStringLiteral("confidence_score"), report.confidenceScore},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)},
            {QStringLiteral("message"), report.message}};
}

QJsonObject pipelineStageToJson(const PipelineStageReport &stage)
{
    return {{QStringLiteral("name"), stage.name},
            {QStringLiteral("state"), pipelineStageStateName(stage.state)},
            {QStringLiteral("message"), stage.message}};
}

QJsonObject resultToJson(const CalibrationResult &result)
{
    return {{QStringLiteral("method"), methodName(result.method)},
            {QStringLiteral("success"), result.success},
            {QStringLiteral("recommended"), result.recommended},
            {QStringLiteral("camera_to_gripper"), matrixToJson(result.cameraToGripper)},
            {QStringLiteral("rotation_rmse_deg"), result.rotationErrorDeg},
            {QStringLiteral("translation_rmse_m"), result.translationError},
            {QStringLiteral("elapsed_ms"), result.elapsedMs},
            {QStringLiteral("message"), result.message},
            {QStringLiteral("training_report"), reportToJson(result.trainingReport)},
            {QStringLiteral("validation_report"), reportToJson(result.validationReport)},
            {QStringLiteral("fixed_target_report"), fixedTargetReportToJson(result.fixedTargetReport)},
            {QStringLiteral("fixed_point_report"), fixedPointReportToJson(result.fixedPointReport)},
            {QStringLiteral("quality_report"), qualityReportToJson(result.qualityReport)},
            {QStringLiteral("optimization_report"), optimizationReportToJson(result.optimizationReport)},
            {QStringLiteral("bootstrap_report"), bootstrapReportToJson(result.bootstrapReport)}};
}

CalibrationMethod methodFromString(const QString &value)
{
    if (value == QStringLiteral("Park-Martin")) return CalibrationMethod::Park;
    if (value == QStringLiteral("Horaud")) return CalibrationMethod::Horaud;
    if (value == QStringLiteral("Andreff")) return CalibrationMethod::Andreff;
    if (value == QStringLiteral("Daniilidis")) return CalibrationMethod::Daniilidis;
    if (value.startsWith(QStringLiteral("FixedPoint3D"))) return CalibrationMethod::PointBased;
    if (value.startsWith(QStringLiteral("非线性"))) return CalibrationMethod::Nonlinear;
    return CalibrationMethod::Tsai;
}

CalibrationResult resultFromJson(const QJsonObject &object)
{
    CalibrationResult result;
    result.method = methodFromString(object.value(QStringLiteral("method")).toString());
    result.success = object.value(QStringLiteral("success")).toBool();
    result.recommended = object.value(QStringLiteral("recommended")).toBool();
    result.cameraToGripper = matrixFromJson(object.value(QStringLiteral("camera_to_gripper")));
    result.rotationErrorDeg = object.value(QStringLiteral("rotation_rmse_deg")).toDouble();
    result.translationError = object.value(QStringLiteral("translation_rmse_m")).toDouble();
    result.elapsedMs = object.value(QStringLiteral("elapsed_ms")).toInteger();
    result.message = object.value(QStringLiteral("message")).toString();
    result.trainingReport = reportFromJson(object.value(QStringLiteral("training_report")).toObject());
    result.validationReport = reportFromJson(object.value(QStringLiteral("validation_report")).toObject());
    result.fixedTargetReport = fixedTargetReportFromJson(object.value(QStringLiteral("fixed_target_report")).toObject());
    result.fixedPointReport = fixedPointReportFromJson(object.value(QStringLiteral("fixed_point_report")).toObject());
    result.qualityReport = qualityReportFromJson(object.value(QStringLiteral("quality_report")).toObject());
    result.optimizationReport = optimizationReportFromJson(object.value(QStringLiteral("optimization_report")).toObject());
    result.bootstrapReport = bootstrapReportFromJson(object.value(QStringLiteral("bootstrap_report")).toObject());
    return result;
}

QJsonObject reliabilityPipelineReportToJson(const ReliabilityPipelineReport &report)
{
    QJsonArray stages;
    for (const PipelineStageReport &stage : report.stages) stages.append(pipelineStageToJson(stage));
    QJsonArray removedIds;
    for (int id : report.removedSampleIds) removedIds.append(id);
    return {{QStringLiteral("available"), report.available}, {QStringLiteral("success"), report.success},
            {QStringLiteral("passed"), report.passed},
            {QStringLiteral("initial_sample_count"), report.initialSampleCount},
            {QStringLiteral("final_sample_count"), report.finalSampleCount},
            {QStringLiteral("auto_removed_count"), report.autoRemovedCount},
            {QStringLiteral("removed_sample_ids"), removedIds}, {QStringLiteral("stages"), stages},
            {QStringLiteral("pnp_report"), pnpQualityReportToJson(report.pnpReport)},
            {QStringLiteral("ax_xb_report"), reportToJson(report.axXbReport)},
            {QStringLiteral("fixed_target_report"), fixedTargetReportToJson(report.fixedTargetReport)},
            {QStringLiteral("fixed_point_report"), fixedPointReportToJson(report.fixedPointReport)},
            {QStringLiteral("quality_report"), qualityReportToJson(report.qualityReport)},
            {QStringLiteral("optimization_report"), optimizationReportToJson(report.optimizationReport)},
            {QStringLiteral("bootstrap_report"), bootstrapReportToJson(report.bootstrapReport)},
            {QStringLiteral("final_method"), methodName(report.finalMethod)},
            {QStringLiteral("final_camera_to_gripper"), matrixToJson(report.finalCameraToGripper)},
            {QStringLiteral("errors"), QJsonArray::fromStringList(report.errors)},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)},
            {QStringLiteral("message"), report.message},
            {QStringLiteral("completed_at"), report.completedAt.toString(Qt::ISODate)},
            {QStringLiteral("elapsed_ms"), report.elapsedMs}};
}

ReliabilityPipelineReport reliabilityPipelineReportFromJson(const QJsonObject &object)
{
    ReliabilityPipelineReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.success = object.value(QStringLiteral("success")).toBool();
    report.passed = object.value(QStringLiteral("passed")).toBool();
    report.initialSampleCount = object.value(QStringLiteral("initial_sample_count")).toInt();
    report.finalSampleCount = object.value(QStringLiteral("final_sample_count")).toInt();
    report.autoRemovedCount = object.value(QStringLiteral("auto_removed_count")).toInt();
    for (const QJsonValue &value : object.value(QStringLiteral("removed_sample_ids")).toArray())
        report.removedSampleIds.append(value.toInt());
    for (const QJsonValue &value : object.value(QStringLiteral("stages")).toArray()) {
        const QJsonObject stage = value.toObject();
        report.stages.append({stage.value(QStringLiteral("name")).toString(),
                              pipelineStageStateFromName(stage.value(QStringLiteral("state")).toString()),
                              stage.value(QStringLiteral("message")).toString()});
    }
    report.pnpReport = pnpQualityReportFromJson(object.value(QStringLiteral("pnp_report")).toObject());
    report.axXbReport = reportFromJson(object.value(QStringLiteral("ax_xb_report")).toObject());
    report.fixedTargetReport = fixedTargetReportFromJson(object.value(QStringLiteral("fixed_target_report")).toObject());
    report.fixedPointReport = fixedPointReportFromJson(object.value(QStringLiteral("fixed_point_report")).toObject());
    report.qualityReport = qualityReportFromJson(object.value(QStringLiteral("quality_report")).toObject());
    report.optimizationReport = optimizationReportFromJson(object.value(QStringLiteral("optimization_report")).toObject());
    report.bootstrapReport = bootstrapReportFromJson(object.value(QStringLiteral("bootstrap_report")).toObject());
    report.finalMethod = methodFromString(object.value(QStringLiteral("final_method")).toString());
    report.finalCameraToGripper = matrixFromJson(object.value(QStringLiteral("final_camera_to_gripper")));
    for (const QJsonValue &value : object.value(QStringLiteral("errors")).toArray()) report.errors.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray()) report.warnings.append(value.toString());
    report.message = object.value(QStringLiteral("message")).toString();
    report.completedAt = QDateTime::fromString(object.value(QStringLiteral("completed_at")).toString(), Qt::ISODate);
    report.elapsedMs = object.value(QStringLiteral("elapsed_ms")).toInteger();
    return report;
}

QJsonObject intrinsicsToJson(const CameraIntrinsics &intrinsics)
{
    return {{QStringLiteral("valid"), intrinsics.valid},
            {QStringLiteral("camera_matrix"), matrix3ToJson(intrinsics.cameraMatrix)},
            {QStringLiteral("distortion_coeffs"), vector5ToJson(intrinsics.distortionCoeffs)},
            {QStringLiteral("image_width"), intrinsics.imageWidth},
            {QStringLiteral("image_height"), intrinsics.imageHeight},
            {QStringLiteral("source"), intrinsics.source},
            {QStringLiteral("calibrated_at"), intrinsics.calibratedAt.toString(Qt::ISODate)}};
}

CameraIntrinsics intrinsicsFromJson(const QJsonObject &object)
{
    CameraIntrinsics intrinsics;
    intrinsics.valid = object.value(QStringLiteral("valid")).toBool(false);
    intrinsics.cameraMatrix = matrix3FromJson(object.value(QStringLiteral("camera_matrix")));
    intrinsics.distortionCoeffs = vector5FromJson(object.value(QStringLiteral("distortion_coeffs")));
    intrinsics.imageWidth = object.value(QStringLiteral("image_width")).toInt();
    intrinsics.imageHeight = object.value(QStringLiteral("image_height")).toInt();
    intrinsics.source = object.value(QStringLiteral("source")).toString();
    intrinsics.calibratedAt = QDateTime::fromString(object.value(QStringLiteral("calibrated_at")).toString(), Qt::ISODate);
    return intrinsics;
}

QJsonObject cameraSampleToJson(const CameraCalibrationSample &sample)
{
    QJsonArray markerCorners;
    for (const QVector<Vector2> &marker : sample.markerCorners) {
        QJsonArray corners;
        for (const Vector2 &corner : marker) corners.append(vector2ToJson(corner));
        markerCorners.append(corners);
    }
    return {{QStringLiteral("image_path"), sample.imagePath},
            {QStringLiteral("image_width"), sample.imageWidth},
            {QStringLiteral("image_height"), sample.imageHeight},
            {QStringLiteral("detected_corner_count"), sample.detectedCornerCount},
            {QStringLiteral("reprojection_rmse_px"), sample.reprojectionRmsePx},
            {QStringLiteral("used"), sample.used},
            {QStringLiteral("outlier"), sample.outlier},
            {QStringLiteral("status"), cameraCalibrationSampleStatusName(sample.status)},
            {QStringLiteral("message"), sample.message},
            {QStringLiteral("corner_ids"), QJsonArray::fromVariantList([&] {
                 QVariantList values;
                 for (int id : sample.cornerIds) values.append(id);
                 return values;
             }())},
            {QStringLiteral("detection_method"), sample.detectionMethod},
            {QStringLiteral("marker_corners"), markerCorners}};
}

CameraCalibrationSample cameraSampleFromJson(const QJsonObject &object)
{
    CameraCalibrationSample sample;
    sample.imagePath = object.value(QStringLiteral("image_path")).toString();
    sample.imageWidth = object.value(QStringLiteral("image_width")).toInt();
    sample.imageHeight = object.value(QStringLiteral("image_height")).toInt();
    sample.detectedCornerCount = object.value(QStringLiteral("detected_corner_count")).toInt();
    sample.reprojectionRmsePx = object.value(QStringLiteral("reprojection_rmse_px")).toDouble();
    sample.used = object.value(QStringLiteral("used")).toBool();
    sample.outlier = object.value(QStringLiteral("outlier")).toBool();
    sample.status = cameraCalibrationSampleStatusFromName(object.value(QStringLiteral("status")).toString());
    sample.message = object.value(QStringLiteral("message")).toString();
    for (const QJsonValue &value : object.value(QStringLiteral("corner_ids")).toArray())
        sample.cornerIds.append(value.toInt());
    sample.detectionMethod = object.value(QStringLiteral("detection_method")).toString();
    for (const QJsonValue &markerValue : object.value(QStringLiteral("marker_corners")).toArray()) {
        QVector<Vector2> marker;
        for (const QJsonValue &cornerValue : markerValue.toArray()) marker.append(vector2FromJson(cornerValue));
        sample.markerCorners.append(marker);
    }
    return sample;
}

QJsonObject cameraReportToJson(const CameraCalibrationReport &report)
{
    QJsonArray samples;
    for (const CameraCalibrationSample &sample : report.samples) samples.append(cameraSampleToJson(sample));
    return {{QStringLiteral("available"), report.available},
            {QStringLiteral("success"), report.success},
            {QStringLiteral("passed"), report.passed},
            {QStringLiteral("initial_image_count"), report.initialImageCount},
            {QStringLiteral("initial_detected_count"), report.initialDetectedCount},
            {QStringLiteral("final_used_count"), report.finalUsedCount},
            {QStringLiteral("outlier_count"), report.outlierCount},
            {QStringLiteral("image_width"), report.imageWidth},
            {QStringLiteral("image_height"), report.imageHeight},
            {QStringLiteral("outlier_threshold_px"), report.outlierThresholdPx},
            {QStringLiteral("rms_px"), report.rmsPx},
            {QStringLiteral("mean_rmse_px"), report.meanRmsePx},
            {QStringLiteral("max_rmse_px"), report.maxRmsePx},
            {QStringLiteral("coverage_warning"), report.coverageWarning},
            {QStringLiteral("errors"), QJsonArray::fromStringList(report.errors)},
            {QStringLiteral("warnings"), QJsonArray::fromStringList(report.warnings)},
            {QStringLiteral("message"), report.message},
            {QStringLiteral("calibrated_at"), report.calibratedAt.toString(Qt::ISODate)},
            {QStringLiteral("intrinsics"), intrinsicsToJson(report.intrinsics)},
            {QStringLiteral("samples"), samples}};
}

CameraCalibrationReport cameraReportFromJson(const QJsonObject &object)
{
    CameraCalibrationReport report;
    report.available = object.value(QStringLiteral("available")).toBool();
    report.success = object.value(QStringLiteral("success")).toBool();
    report.passed = object.value(QStringLiteral("passed")).toBool();
    report.initialImageCount = object.value(QStringLiteral("initial_image_count")).toInt();
    report.initialDetectedCount = object.value(QStringLiteral("initial_detected_count")).toInt();
    report.finalUsedCount = object.value(QStringLiteral("final_used_count")).toInt();
    report.outlierCount = object.value(QStringLiteral("outlier_count")).toInt();
    report.imageWidth = object.value(QStringLiteral("image_width")).toInt();
    report.imageHeight = object.value(QStringLiteral("image_height")).toInt();
    report.outlierThresholdPx = object.value(QStringLiteral("outlier_threshold_px")).toDouble(1.0);
    report.rmsPx = object.value(QStringLiteral("rms_px")).toDouble();
    report.meanRmsePx = object.value(QStringLiteral("mean_rmse_px")).toDouble();
    report.maxRmsePx = object.value(QStringLiteral("max_rmse_px")).toDouble();
    report.coverageWarning = object.value(QStringLiteral("coverage_warning")).toBool();
    for (const QJsonValue &value : object.value(QStringLiteral("errors")).toArray()) report.errors.append(value.toString());
    for (const QJsonValue &value : object.value(QStringLiteral("warnings")).toArray()) report.warnings.append(value.toString());
    report.message = object.value(QStringLiteral("message")).toString();
    report.calibratedAt = QDateTime::fromString(object.value(QStringLiteral("calibrated_at")).toString(), Qt::ISODate);
    report.intrinsics = intrinsicsFromJson(object.value(QStringLiteral("intrinsics")).toObject());
    for (const QJsonValue &value : object.value(QStringLiteral("samples")).toArray())
        report.samples.append(cameraSampleFromJson(value.toObject()));
    return report;
}

QString vectorText(const Vector3 &value)
{
    return QStringLiteral("[%1, %2, %3]").arg(value[0], 0, 'g', 15)
        .arg(value[1], 0, 'g', 15).arg(value[2], 0, 'g', 15);
}

QString matrixText(const Matrix4 &matrix)
{
    QStringList lines;
    for (const auto &row : matrix)
        lines << QStringLiteral("[%1, %2, %3, %4]").arg(row[0], 0, 'g', 15)
            .arg(row[1], 0, 'g', 15).arg(row[2], 0, 'g', 15).arg(row[3], 0, 'g', 15);
    return lines.join('\n');
}

} // namespace

IoResult writeCsv(const QString &filePath, const CalibrationDataset &dataset)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return {false, file.errorString()};
    QTextStream stream(&file);
    stream << "# handeye_schema=2; rotation=Rodrigues; angle=rad; length=m; "
              "gripper=gripper_to_base; target=target_to_camera\n";
    stream << "id,label,gripper_rx,gripper_ry,gripper_rz,gripper_tx,gripper_ty,gripper_tz,"
              "target_rx,target_ry,target_rz,target_tx,target_ty,target_tz\n";
    for (const PoseSample &sample : dataset.samples) {
        stream << sample.id << ',' << sample.label << ',';
        for (double value : sample.gripperRotation) stream << value << ',';
        for (double value : sample.gripperTranslation) stream << value << ',';
        for (double value : sample.targetRotation) stream << value << ',';
        stream << sample.targetTranslation[0] << ',' << sample.targetTranslation[1] << ','
               << sample.targetTranslation[2] << '\n';
    }
    return {true, {}};
}

IoResult readCsv(const QString &filePath, CalibrationDataset *dataset, const PoseInputSpec &inputSpec)
{
    if (!dataset) return {false, QStringLiteral("目标数据集为空。")};
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return {false, file.errorString()};
    QTextStream stream(&file);
    QString header = stream.readLine();
    while (header.startsWith(QLatin1Char('#')) && !stream.atEnd()) header = stream.readLine();
    if (!header.startsWith(QStringLiteral("id,label"))) return {false, QStringLiteral("CSV 表头不符合要求。")};
    const int rotationCount = inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ ? 4 : 3;
    const int expectedFields = 2 + rotationCount + 3 + rotationCount + 3;
    QVector<PoseSample> samples;
    int lineNumber = 1;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) continue;
        const QStringList fields = line.split(',');
        if (fields.size() < expectedFields)
            return {false, QStringLiteral("第 %1 行列数不足，应为至少 %2 列。")
                        .arg(lineNumber).arg(expectedFields)};
        bool ok = false;
        PoseSample sample;
        sample.id = fields.at(0).toInt(&ok);
        if (!ok) return {false, QStringLiteral("第 %1 行 ID 无效。").arg(lineNumber)};
        sample.label = fields.at(1);
        QVector<double> values;
        for (int index = 2; index < expectedFields; ++index) {
            const double value = fields.at(index).toDouble(&ok);
            if (!ok) return {false, QStringLiteral("第 %1 行第 %2 列不是数字。")
                                .arg(lineNumber).arg(index + 1)};
            values.append(value);
        }
        Vector4 gripperRotation{0, 0, 0, 0};
        Vector4 targetRotation{0, 0, 0, 0};
        for (int index = 0; index < rotationCount; ++index) gripperRotation[index] = values[index];
        Vector3 gripperTranslation{values[rotationCount], values[rotationCount + 1], values[rotationCount + 2]};
        const int targetOffset = rotationCount + 3;
        for (int index = 0; index < rotationCount; ++index) targetRotation[index] = values[targetOffset + index];
        const int targetTranslationOffset = targetOffset + rotationCount;
        Vector3 targetTranslation{values[targetTranslationOffset], values[targetTranslationOffset + 1],
                                  values[targetTranslationOffset + 2]};
        const auto gripper = pose::normalize(gripperRotation, gripperTranslation, inputSpec);
        const auto target = pose::normalize(targetRotation, targetTranslation, inputSpec);
        if (!gripper.success || !target.success)
            return {false, QStringLiteral("第 %1 行姿态标准化失败：%2 %3")
                        .arg(lineNumber).arg(gripper.error, target.error)};
        sample.gripperRotation = gripper.rotation;
        sample.gripperTranslation = gripper.translation;
        sample.targetRotation = target.rotation;
        sample.targetTranslation = target.translation;
        samples.append(sample);
    }
    dataset->samples = samples;
    dataset->pointSamples.clear();
    dataset->inputMode = CalibrationInputMode::PosePairs;
    dataset->targetPosesReady = true;
    dataset->inputSpec = inputSpec;
    dataset->results.clear();
    return {true, {}};
}

IoResult writeJson(const QString &filePath, const CalibrationDataset &dataset)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) return {false, file.errorString()};
    QJsonArray samples;
    for (const PoseSample &sample : dataset.samples) samples.append(sampleToJson(sample));
    QJsonArray validationSamples;
    for (const PoseSample &sample : dataset.validationSamples) validationSamples.append(sampleToJson(sample));
    QJsonArray pointSamples;
    for (const PointSample &sample : dataset.pointSamples) pointSamples.append(pointSampleToJson(sample));
    QJsonArray results;
    for (const CalibrationResult &result : dataset.results) results.append(resultToJson(result));
    const QJsonObject root{
        {QStringLiteral("schema_version"), 4}, {QStringLiteral("mode"), modeToString(dataset.mode)},
        {QStringLiteral("input_mode"), inputModeToString(dataset.inputMode)},
        {QStringLiteral("rotation_format"), formatToString(dataset.inputSpec.rotationFormat)},
        {QStringLiteral("angle_unit"), angleUnitName(dataset.inputSpec.angleUnit)},
        {QStringLiteral("length_unit"), lengthUnitName(dataset.inputSpec.lengthUnit)},
        {QStringLiteral("pose_adapter"), adapterToString(dataset.inputSpec.adapter)},
        {QStringLiteral("direction_gripper"), directionName(PoseDirection::GripperToBase)},
        {QStringLiteral("direction_target"), directionName(PoseDirection::TargetToCamera)},
        {QStringLiteral("direction_result"), directionName(PoseDirection::CameraToGripper)},
        {QStringLiteral("robot"), dataset.robotName}, {QStringLiteral("camera"), dataset.cameraName},
        {QStringLiteral("notes"), dataset.notes},
        {QStringLiteral("board_pattern"), boardPatternName(dataset.boardSpec.pattern)},
        {QStringLiteral("chessboard_detector"), chessboardDetectorName(dataset.boardSpec.chessboardDetector)},
        {QStringLiteral("pnp_method"), pnpMethodName(dataset.boardSpec.pnpMethod)},
        {QStringLiteral("board_inner_corners_x"), dataset.boardSpec.innerCornersX},
        {QStringLiteral("board_inner_corners_y"), dataset.boardSpec.innerCornersY},
        {QStringLiteral("board_square_size_m"), dataset.boardSpec.squareSizeM},
        {QStringLiteral("aruco_dictionary"), dataset.boardSpec.arucoDictionary},
        {QStringLiteral("marker_count_x"), dataset.boardSpec.markerCountX},
        {QStringLiteral("marker_count_y"), dataset.boardSpec.markerCountY},
        {QStringLiteral("marker_size_m"), dataset.boardSpec.markerSizeM},
        {QStringLiteral("camera_intrinsics_valid"), dataset.cameraIntrinsics.valid},
        {QStringLiteral("camera_matrix"), matrix3ToJson(dataset.cameraIntrinsics.cameraMatrix)},
        {QStringLiteral("distortion_coeffs"), vector5ToJson(dataset.cameraIntrinsics.distortionCoeffs)},
        {QStringLiteral("camera_image_width"), dataset.cameraIntrinsics.imageWidth},
        {QStringLiteral("camera_image_height"), dataset.cameraIntrinsics.imageHeight},
        {QStringLiteral("intrinsics_source"), dataset.cameraIntrinsics.source},
        {QStringLiteral("intrinsics_calibrated_at"), dataset.cameraIntrinsics.calibratedAt.toString(Qt::ISODate)},
        {QStringLiteral("camera_calibration_report"), cameraReportToJson(dataset.cameraCalibrationReport)},
        {QStringLiteral("pass_rotation_rmse_deg"), dataset.passRotationRmseDeg},
        {QStringLiteral("pass_translation_rmse_m"), dataset.passTranslationRmseM},
        {QStringLiteral("bootstrap_resamples"), dataset.bootstrapResamples},
        {QStringLiteral("bootstrap_confidence"), dataset.bootstrapConfidence},
        {QStringLiteral("created_at"), dataset.createdAt.toString(Qt::ISODate)},
        {QStringLiteral("target_poses_ready"), dataset.targetPosesReady},
        {QStringLiteral("samples"), samples}, {QStringLiteral("validation_samples"), validationSamples},
        {QStringLiteral("point_samples"), pointSamples},
        {QStringLiteral("reliability_pipeline_report"), reliabilityPipelineReportToJson(dataset.reliabilityPipelineReport)},
        {QStringLiteral("results"), results}};
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return {true, {}};
}

IoResult readJson(const QString &filePath, CalibrationDataset *dataset)
{
    if (!dataset) return {false, QStringLiteral("目标数据集为空。")};
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) return {false, file.errorString()};
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (document.isNull() || !document.isObject()) return {false, parseError.errorString()};
    const QJsonObject root = document.object();
    CalibrationDataset parsed;
    parsed.mode = modeFromString(root.value(QStringLiteral("mode")).toString());
    parsed.inputMode = inputModeFromString(root.value(QStringLiteral("input_mode")).toString());
    parsed.inputSpec.rotationFormat = formatFromString(root.value(QStringLiteral("rotation_format")).toString());
    parsed.inputSpec.angleUnit = root.value(QStringLiteral("angle_unit")).toString() == QStringLiteral("deg")
                                     ? AngleUnit::Degrees : AngleUnit::Radians;
    parsed.inputSpec.lengthUnit = root.value(QStringLiteral("length_unit")).toString() == QStringLiteral("mm")
                                      ? LengthUnit::Millimeters : LengthUnit::Meters;
    parsed.inputSpec.adapter = adapterFromString(root.value(QStringLiteral("pose_adapter")).toString());
    parsed.robotName = root.value(QStringLiteral("robot")).toString(parsed.robotName);
    parsed.cameraName = root.value(QStringLiteral("camera")).toString(parsed.cameraName);
    parsed.notes = root.value(QStringLiteral("notes")).toString();
    parsed.boardSpec.innerCornersX = root.value(QStringLiteral("board_inner_corners_x"))
                                         .toInt(parsed.boardSpec.innerCornersX);
    parsed.boardSpec.pattern = boardPatternFromString(root.value(QStringLiteral("board_pattern")).toString());
    parsed.boardSpec.chessboardDetector = detectorFromString(root.value(QStringLiteral("chessboard_detector")).toString());
    parsed.boardSpec.pnpMethod = pnpFromString(root.value(QStringLiteral("pnp_method")).toString());
    parsed.boardSpec.innerCornersY = root.value(QStringLiteral("board_inner_corners_y"))
                                         .toInt(parsed.boardSpec.innerCornersY);
    parsed.boardSpec.squareSizeM = root.value(QStringLiteral("board_square_size_m"))
                                       .toDouble(parsed.boardSpec.squareSizeM);
    parsed.boardSpec.arucoDictionary = root.value(QStringLiteral("aruco_dictionary"))
                                           .toInt(parsed.boardSpec.arucoDictionary);
    parsed.boardSpec.markerCountX = root.value(QStringLiteral("marker_count_x"))
                                        .toInt(parsed.boardSpec.markerCountX);
    parsed.boardSpec.markerCountY = root.value(QStringLiteral("marker_count_y"))
                                        .toInt(parsed.boardSpec.markerCountY);
    parsed.boardSpec.markerSizeM = root.value(QStringLiteral("marker_size_m"))
                                       .toDouble(parsed.boardSpec.markerSizeM);
    parsed.cameraIntrinsics.valid = root.value(QStringLiteral("camera_intrinsics_valid")).toBool(false);
    parsed.cameraIntrinsics.cameraMatrix = matrix3FromJson(root.value(QStringLiteral("camera_matrix")));
    parsed.cameraIntrinsics.distortionCoeffs = vector5FromJson(root.value(QStringLiteral("distortion_coeffs")));
    parsed.cameraIntrinsics.imageWidth = root.value(QStringLiteral("camera_image_width")).toInt();
    parsed.cameraIntrinsics.imageHeight = root.value(QStringLiteral("camera_image_height")).toInt();
    parsed.cameraIntrinsics.source = root.value(QStringLiteral("intrinsics_source")).toString();
    parsed.cameraIntrinsics.calibratedAt = QDateTime::fromString(
        root.value(QStringLiteral("intrinsics_calibrated_at")).toString(), Qt::ISODate);
    parsed.cameraCalibrationReport = cameraReportFromJson(
        root.value(QStringLiteral("camera_calibration_report")).toObject());
    parsed.passRotationRmseDeg = root.value(QStringLiteral("pass_rotation_rmse_deg")).toDouble(parsed.passRotationRmseDeg);
    parsed.passTranslationRmseM = root.value(QStringLiteral("pass_translation_rmse_m")).toDouble(parsed.passTranslationRmseM);
    parsed.bootstrapResamples = root.value(QStringLiteral("bootstrap_resamples")).toInt(parsed.bootstrapResamples);
    parsed.bootstrapConfidence = root.value(QStringLiteral("bootstrap_confidence")).toDouble(parsed.bootstrapConfidence);
    parsed.createdAt = QDateTime::fromString(root.value(QStringLiteral("created_at")).toString(), Qt::ISODate);
    for (const QJsonValue &value : root.value(QStringLiteral("samples")).toArray())
        parsed.samples.append(sampleFromJson(value.toObject()));
    for (const QJsonValue &value : root.value(QStringLiteral("validation_samples")).toArray())
        parsed.validationSamples.append(sampleFromJson(value.toObject()));
    for (const QJsonValue &value : root.value(QStringLiteral("point_samples")).toArray())
        parsed.pointSamples.append(pointSampleFromJson(value.toObject()));
    for (const QJsonValue &value : root.value(QStringLiteral("results")).toArray())
        parsed.results.append(resultFromJson(value.toObject()));
    parsed.reliabilityPipelineReport = reliabilityPipelineReportFromJson(
        root.value(QStringLiteral("reliability_pipeline_report")).toObject());
    parsed.targetPosesReady = !parsed.samples.isEmpty()
                              && root.value(QStringLiteral("target_poses_ready")).toBool(true);
    *dataset = parsed;
    return {true, {}};
}

IoResult writeYaml(const QString &filePath, const CalibrationDataset &dataset)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return {false, file.errorString()};
    QTextStream stream(&file);
    stream << "schema_version: 3\nmode: " << modeToString(dataset.mode) << "\n"
           << "input_mode: " << inputModeToString(dataset.inputMode) << "\n"
           << "robot: \"" << dataset.robotName << "\"\ncamera: \"" << dataset.cameraName << "\"\n"
           << "rotation_format: \"" << formatToString(dataset.inputSpec.rotationFormat) << "\"\n"
           << "board_pattern: \"" << boardPatternName(dataset.boardSpec.pattern) << "\"\n"
           << "chessboard_detector: \"" << chessboardDetectorName(dataset.boardSpec.chessboardDetector) << "\"\n"
           << "pnp_method: \"" << pnpMethodName(dataset.boardSpec.pnpMethod) << "\"\n"
           << "angle_unit: " << angleUnitName(dataset.inputSpec.angleUnit) << "\nlength_unit: "
           << lengthUnitName(dataset.inputSpec.lengthUnit) << "\ncreated_at: \""
           << dataset.createdAt.toString(Qt::ISODate) << "\"\n"
           << "camera_intrinsics_valid: " << (dataset.cameraIntrinsics.valid ? "true" : "false") << "\n"
           << "camera_image_width: " << dataset.cameraIntrinsics.imageWidth << "\n"
           << "camera_image_height: " << dataset.cameraIntrinsics.imageHeight << "\n"
           << "intrinsics_source: \"" << dataset.cameraIntrinsics.source << "\"\n"
           << "intrinsics_calibrated_at: \"" << dataset.cameraIntrinsics.calibratedAt.toString(Qt::ISODate)
           << "\"\ncamera_matrix:\n";
    for (const auto &row : dataset.cameraIntrinsics.cameraMatrix)
        stream << "  - [" << row[0] << ", " << row[1] << ", " << row[2] << "]\n";
    stream << "distortion_coeffs: [";
    for (int index = 0; index < 5; ++index) {
        if (index > 0) stream << ", ";
        stream << dataset.cameraIntrinsics.distortionCoeffs[index];
    }
    stream << "]\ncamera_calibration:\n"
           << "  available: " << (dataset.cameraCalibrationReport.available ? "true" : "false") << "\n"
           << "  success: " << (dataset.cameraCalibrationReport.success ? "true" : "false") << "\n"
           << "  passed: " << (dataset.cameraCalibrationReport.passed ? "true" : "false") << "\n"
           << "  initial_image_count: " << dataset.cameraCalibrationReport.initialImageCount << "\n"
           << "  final_used_count: " << dataset.cameraCalibrationReport.finalUsedCount << "\n"
           << "  outlier_count: " << dataset.cameraCalibrationReport.outlierCount << "\n"
           << "  rms_px: " << dataset.cameraCalibrationReport.rmsPx << "\n"
           << "  mean_rmse_px: " << dataset.cameraCalibrationReport.meanRmsePx << "\n"
           << "  max_rmse_px: " << dataset.cameraCalibrationReport.maxRmsePx << "\n"
           << "  coverage_warning: " << (dataset.cameraCalibrationReport.coverageWarning ? "true" : "false") << "\n"
           << "  calibrated_at: \"" << dataset.cameraCalibrationReport.calibratedAt.toString(Qt::ISODate)
           << "\"\n"
           << "pass_rotation_rmse_deg: " << dataset.passRotationRmseDeg << "\n"
           << "pass_translation_rmse_m: " << dataset.passTranslationRmseM << "\n"
           << "bootstrap_resamples: " << dataset.bootstrapResamples << "\n"
           << "bootstrap_confidence: " << dataset.bootstrapConfidence << "\n"
           << "reliability_pipeline:\n"
           << "  available: " << (dataset.reliabilityPipelineReport.available ? "true" : "false") << "\n"
           << "  success: " << (dataset.reliabilityPipelineReport.success ? "true" : "false") << "\n"
           << "  passed: " << (dataset.reliabilityPipelineReport.passed ? "true" : "false") << "\n"
           << "  initial_sample_count: " << dataset.reliabilityPipelineReport.initialSampleCount << "\n"
           << "  final_sample_count: " << dataset.reliabilityPipelineReport.finalSampleCount << "\n"
           << "  auto_removed_count: " << dataset.reliabilityPipelineReport.autoRemovedCount << "\n"
           << "  final_method: " << methodName(dataset.reliabilityPipelineReport.finalMethod) << "\n"
           << "  final_camera_to_gripper:\n";
    for (const auto &row : dataset.reliabilityPipelineReport.finalCameraToGripper)
        stream << "    - [" << row[0] << ", " << row[1] << ", " << row[2] << ", " << row[3] << "]\n";
    stream << "  stages:\n";
    for (const PipelineStageReport &stage : dataset.reliabilityPipelineReport.stages)
        stream << "    - name: \"" << stage.name << "\"\n"
               << "      state: \"" << pipelineStageStateName(stage.state) << "\"\n"
               << "      message: \"" << stage.message << "\"\n";
    stream << "  bootstrap:\n"
           << "    requested_resamples: " << dataset.reliabilityPipelineReport.bootstrapReport.requestedResamples << "\n"
           << "    successful_resamples: " << dataset.reliabilityPipelineReport.bootstrapReport.successfulResamples << "\n"
           << "    confidence_level: " << dataset.reliabilityPipelineReport.bootstrapReport.confidenceLevel << "\n"
           << "    confidence_score: " << dataset.reliabilityPipelineReport.bootstrapReport.confidenceScore << "\n"
           << "    rotation_norm_std_deg: " << dataset.reliabilityPipelineReport.bootstrapReport.rotationNormStdDeg << "\n"
           << "    translation_norm_std_m: " << dataset.reliabilityPipelineReport.bootstrapReport.translationNormStdM << "\n"
           << "point_samples:\n";
    for (const PointSample &sample : dataset.pointSamples) {
        stream << "  - id: " << sample.id << "\n"
               << "    gripper_rotation_rodrigues_rad: " << vectorText(sample.gripperRotation) << "\n"
               << "    gripper_translation_m: " << vectorText(sample.gripperTranslation) << "\n"
               << "    camera_point_m: " << vectorText(sample.cameraPoint) << "\n"
               << "    residual_m: " << sample.residualM << "\n"
               << "    outlier: " << (sample.outlier ? "true" : "false") << "\n";
    }
    stream << "results:\n";
    for (const CalibrationResult &result : dataset.results) {
        stream << "  - method: " << methodName(result.method) << "\n"
               << "    success: " << (result.success ? "true" : "false") << "\n"
               << "    recommended: " << (result.recommended ? "true" : "false") << "\n"
               << "    camera_to_gripper:\n";
        for (const auto &row : result.cameraToGripper)
            stream << "      - [" << row[0] << ", " << row[1] << ", " << row[2] << ", " << row[3] << "]\n";
        stream << "    rotation_rmse_deg: " << result.trainingReport.rotationRmseDeg << "\n"
               << "    translation_rmse_m: " << result.trainingReport.translationRmseM << "\n"
               << "    rotation_max_deg: " << result.trainingReport.rotationMaxDeg << "\n"
               << "    translation_max_m: " << result.trainingReport.translationMaxM << "\n"
               << "    quality_score: " << result.qualityReport.totalScore << "\n"
               << "    quality_level: \"" << result.qualityReport.level << "\"\n"
               << "    optimization_before_translation_rmse_m: " << result.optimizationReport.beforeTranslationRmseM << "\n"
               << "    optimization_after_translation_rmse_m: " << result.optimizationReport.afterTranslationRmseM << "\n"
               << "    bootstrap_successful_resamples: " << result.bootstrapReport.successfulResamples << "\n"
               << "    bootstrap_confidence_score: " << result.bootstrapReport.confidenceScore << "\n"
               << "    bootstrap_rotation_norm_std_deg: " << result.bootstrapReport.rotationNormStdDeg << "\n"
               << "    bootstrap_translation_norm_std_m: " << result.bootstrapReport.translationNormStdM << "\n"
               << "    passed: " << (result.trainingReport.passed ? "true" : "false") << "\n";
    }
    return {true, {}};
}

IoResult writeResultTxt(const QString &filePath, const CalibrationDataset &dataset,
                        const CalibrationResult &result)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return {false, file.errorString()};
    QTextStream stream(&file);
    stream << "Hand-Eye Calibration Result\nmethod: " << methodName(result.method)
           << "\ninput mode: " << inputModeToString(dataset.inputMode)
           << "\nrobot: " << dataset.robotName << "\ncamera: " << dataset.cameraName
           << "\nresult direction: camera -> gripper\n\n"
           << matrixText(result.cameraToGripper) << "\n\n"
           << "training rotation RMSE (deg): " << result.trainingReport.rotationRmseDeg
           << "\ntraining translation RMSE (m): " << result.trainingReport.translationRmseM
           << "\ntraining rotation mean/max (deg): " << result.trainingReport.rotationMeanDeg << "/"
           << result.trainingReport.rotationMaxDeg << "\ntraining translation mean/max (m): "
           << result.trainingReport.translationMeanM << "/" << result.trainingReport.translationMaxM
           << "\npassed: " << (result.trainingReport.passed ? "true" : "false")
           << "\npose quality score: " << result.qualityReport.totalScore << " ("
           << result.qualityReport.level << ")"
           << "\noptimization translation RMSE before/after (m): "
           << result.optimizationReport.beforeTranslationRmseM << "/"
           << result.optimizationReport.afterTranslationRmseM << '\n';
    return {true, {}};
}

IoResult writeResultCpp(const QString &filePath, const CalibrationDataset &dataset,
                        const CalibrationResult &result)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return {false, file.errorString()};
    QTextStream stream(&file);
    stream << "// Generated by Qt6 Hand-Eye Calibration Tool\n"
           << "// Direction: camera -> gripper\n"
           << "// Input mode: " << inputModeToString(dataset.inputMode) << "\n"
           << "// Robot: " << dataset.robotName << ", Camera: " << dataset.cameraName << "\n"
           << "// Pose quality score: " << result.qualityReport.totalScore << " ("
           << result.qualityReport.level << ")\n"
           << "const cv::Matx44d cameraToGripper = (cv::Matx44d() << ";
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col) stream << result.cameraToGripper[row][col] << (row == 3 && col == 3 ? ")" : ",");
    stream << ";\n";
    return {true, {}};
}

IoResult writeResultPython(const QString &filePath, const CalibrationDataset &dataset,
                           const CalibrationResult &result)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return {false, file.errorString()};
    QTextStream stream(&file);
    stream << "# Generated by Qt6 Hand-Eye Calibration Tool\n"
           << "# Direction: camera -> gripper\n"
           << "# Input mode: " << inputModeToString(dataset.inputMode) << "\n"
           << "# Robot: " << dataset.robotName << ", Camera: " << dataset.cameraName << "\n"
           << "# Pose quality score: " << result.qualityReport.totalScore << " ("
           << result.qualityReport.level << ")\n"
           << "import numpy as np\n\n"
           << "camera_to_gripper = np.array([\n";
    for (const auto &row : result.cameraToGripper)
        stream << "    [" << row[0] << ", " << row[1] << ", " << row[2] << ", " << row[3] << "],\n";
    stream << "], dtype=float)\n";
    return {true, {}};
}

} // namespace handeye
