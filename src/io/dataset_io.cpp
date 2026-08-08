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

Vector3 vectorFromJson(const QJsonValue &value)
{
    const QJsonArray array = value.toArray();
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

CalibrationMode modeFromString(const QString &value)
{
    return value == QStringLiteral("eye_to_hand") ? CalibrationMode::EyeToHand
                                                   : CalibrationMode::EyeInHand;
}

QString formatToString(RotationFormat format) { return rotationFormatName(format); }
QString adapterToString(PoseAdapterKind adapter) { return adapterName(adapter); }

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
            {QStringLiteral("outlier"), sample.outlier}};
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
            {QStringLiteral("validation_report"), reportToJson(result.validationReport)}};
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
    QJsonArray results;
    for (const CalibrationResult &result : dataset.results) results.append(resultToJson(result));
    const QJsonObject root{
        {QStringLiteral("schema_version"), 2}, {QStringLiteral("mode"), modeToString(dataset.mode)},
        {QStringLiteral("rotation_format"), formatToString(dataset.inputSpec.rotationFormat)},
        {QStringLiteral("angle_unit"), angleUnitName(dataset.inputSpec.angleUnit)},
        {QStringLiteral("length_unit"), lengthUnitName(dataset.inputSpec.lengthUnit)},
        {QStringLiteral("pose_adapter"), adapterToString(dataset.inputSpec.adapter)},
        {QStringLiteral("direction_gripper"), directionName(PoseDirection::GripperToBase)},
        {QStringLiteral("direction_target"), directionName(PoseDirection::TargetToCamera)},
        {QStringLiteral("direction_result"), directionName(PoseDirection::CameraToGripper)},
        {QStringLiteral("robot"), dataset.robotName}, {QStringLiteral("camera"), dataset.cameraName},
        {QStringLiteral("notes"), dataset.notes},
        {QStringLiteral("pass_rotation_rmse_deg"), dataset.passRotationRmseDeg},
        {QStringLiteral("pass_translation_rmse_m"), dataset.passTranslationRmseM},
        {QStringLiteral("created_at"), dataset.createdAt.toString(Qt::ISODate)},
        {QStringLiteral("samples"), samples}, {QStringLiteral("validation_samples"), validationSamples},
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
    parsed.inputSpec.rotationFormat = formatFromString(root.value(QStringLiteral("rotation_format")).toString());
    parsed.inputSpec.angleUnit = root.value(QStringLiteral("angle_unit")).toString() == QStringLiteral("deg")
                                     ? AngleUnit::Degrees : AngleUnit::Radians;
    parsed.inputSpec.lengthUnit = root.value(QStringLiteral("length_unit")).toString() == QStringLiteral("mm")
                                      ? LengthUnit::Millimeters : LengthUnit::Meters;
    parsed.inputSpec.adapter = adapterFromString(root.value(QStringLiteral("pose_adapter")).toString());
    parsed.robotName = root.value(QStringLiteral("robot")).toString(parsed.robotName);
    parsed.cameraName = root.value(QStringLiteral("camera")).toString(parsed.cameraName);
    parsed.notes = root.value(QStringLiteral("notes")).toString();
    parsed.passRotationRmseDeg = root.value(QStringLiteral("pass_rotation_rmse_deg")).toDouble(parsed.passRotationRmseDeg);
    parsed.passTranslationRmseM = root.value(QStringLiteral("pass_translation_rmse_m")).toDouble(parsed.passTranslationRmseM);
    parsed.createdAt = QDateTime::fromString(root.value(QStringLiteral("created_at")).toString(), Qt::ISODate);
    for (const QJsonValue &value : root.value(QStringLiteral("samples")).toArray())
        parsed.samples.append(sampleFromJson(value.toObject()));
    for (const QJsonValue &value : root.value(QStringLiteral("validation_samples")).toArray())
        parsed.validationSamples.append(sampleFromJson(value.toObject()));
    *dataset = parsed;
    return {true, {}};
}

IoResult writeYaml(const QString &filePath, const CalibrationDataset &dataset)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return {false, file.errorString()};
    QTextStream stream(&file);
    stream << "schema_version: 2\nmode: " << modeToString(dataset.mode) << "\n"
           << "robot: \"" << dataset.robotName << "\"\ncamera: \"" << dataset.cameraName << "\"\n"
           << "rotation_format: \"" << formatToString(dataset.inputSpec.rotationFormat) << "\"\n"
           << "angle_unit: " << angleUnitName(dataset.inputSpec.angleUnit) << "\nlength_unit: "
           << lengthUnitName(dataset.inputSpec.lengthUnit) << "\ncreated_at: \""
           << dataset.createdAt.toString(Qt::ISODate) << "\"\n"
           << "pass_rotation_rmse_deg: " << dataset.passRotationRmseDeg << "\n"
           << "pass_translation_rmse_m: " << dataset.passTranslationRmseM << "\nresults:\n";
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
           << "\nrobot: " << dataset.robotName << "\ncamera: " << dataset.cameraName
           << "\nresult direction: camera -> gripper\n\n"
           << matrixText(result.cameraToGripper) << "\n\n"
           << "training rotation RMSE (deg): " << result.trainingReport.rotationRmseDeg
           << "\ntraining translation RMSE (m): " << result.trainingReport.translationRmseM
           << "\ntraining rotation mean/max (deg): " << result.trainingReport.rotationMeanDeg << "/"
           << result.trainingReport.rotationMaxDeg << "\ntraining translation mean/max (m): "
           << result.trainingReport.translationMeanM << "/" << result.trainingReport.translationMaxM
           << "\npassed: " << (result.trainingReport.passed ? "true" : "false") << '\n';
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
           << "// Robot: " << dataset.robotName << ", Camera: " << dataset.cameraName << "\n"
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
           << "# Robot: " << dataset.robotName << ", Camera: " << dataset.cameraName << "\n"
           << "import numpy as np\n\n"
           << "camera_to_gripper = np.array([\n";
    for (const auto &row : result.cameraToGripper)
        stream << "    [" << row[0] << ", " << row[1] << ", " << row[2] << ", " << row[3] << "],\n";
    stream << "], dtype=float)\n";
    return {true, {}};
}

} // namespace handeye
