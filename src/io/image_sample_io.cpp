#include "io/image_sample_io.h"

#include "core/pose_conversion.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QTextStream>

namespace handeye {

IoResult readRobotPoseCsv(const QString &filePath, CalibrationDataset *dataset,
                          const PoseInputSpec &inputSpec)
{
    if (!dataset) return {false, QStringLiteral("Dataset output is null.")};
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return {false, file.errorString()};

    QVector<PoseSample> samples;
    QTextStream stream(&file);
    int lineNumber = 0;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#')) continue;
        const QStringList tokens = line.split(QRegularExpression(QStringLiteral(",")), Qt::KeepEmptyParts);
        if (!tokens.isEmpty() && tokens.at(0).trimmed().compare(QStringLiteral("id"), Qt::CaseInsensitive) == 0)
            continue;
        const int expectedTokens = 1 + 3
                                    + (inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ ? 4 : 3);
        if (tokens.size() != expectedTokens)
            return {false, QStringLiteral("Line %1 must contain id, 3 translation values and 3 or 4 rotation values.")
                               .arg(lineNumber)};

        bool idOk = false;
        const int id = tokens.at(0).trimmed().toInt(&idOk);
        if (!idOk) return {false, QStringLiteral("Line %1 has an invalid sample id.").arg(lineNumber)};
        QVector<double> values;
        for (int index = 1; index < tokens.size(); ++index) {
            bool ok = false;
            values.append(tokens.at(index).trimmed().toDouble(&ok));
            if (!ok) return {false, QStringLiteral("Line %1 contains a non-numeric robot pose value.").arg(lineNumber)};
        }

        const Vector3 translation = {values[0], values[1], values[2]};
        const Vector4 rotation = tokens.size() == 8
                                     ? Vector4{values[3], values[4], values[5], values[6]}
                                     : Vector4{values[3], values[4], values[5], 0.0};
        const auto normalized = pose::normalize(rotation, translation, inputSpec);
        if (!normalized.success)
            return {false, QStringLiteral("Line %1 robot pose normalization failed: %2")
                               .arg(lineNumber).arg(normalized.error)};

        PoseSample sample;
        sample.id = id;
        sample.label = QStringLiteral("Pose %1").arg(id);
        sample.gripperRotation = normalized.rotation;
        sample.gripperTranslation = normalized.translation;
        samples.append(sample);
    }
    if (samples.isEmpty()) return {false, QStringLiteral("No robot pose samples found.")};
    dataset->samples = samples;
    dataset->pointSamples.clear();
    dataset->inputMode = CalibrationInputMode::PosePairs;
    dataset->targetPosesReady = false;
    dataset->results.clear();
    return {true, {}};
}

IoResult readPoseImageCsv(const QString &filePath, CalibrationDataset *dataset,
                          const PoseInputSpec &inputSpec)
{
    if (!dataset) return {false, QStringLiteral("Dataset output is null.")};

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return {false, file.errorString()};

    QVector<PoseSample> samples;
    QTextStream stream(&file);
    const QDir csvDirectory = QFileInfo(filePath).absoluteDir();
    int lineNumber = 0;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#')) continue;

        const QStringList tokens = line.split(QRegularExpression(QStringLiteral(",")), Qt::KeepEmptyParts);
        if (tokens.size() >= 2 && tokens.at(0).trimmed().compare(QStringLiteral("id"), Qt::CaseInsensitive) == 0)
            continue;
        const int expectedTokens = 2 + 3
                                    + (inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ ? 4 : 3);
        if (tokens.size() != expectedTokens)
            return {false, QStringLiteral("Line %1 must contain id,image_path,3 translation values and 3 or 4 rotation values.")
                               .arg(lineNumber)};

        bool idOk = false;
        const int id = tokens.at(0).trimmed().toInt(&idOk);
        if (!idOk) return {false, QStringLiteral("Line %1 has an invalid sample id.").arg(lineNumber)};

        bool valuesOk = true;
        QVector<double> values;
        for (int index = 2; index < tokens.size(); ++index) {
            bool ok = false;
            values.append(tokens.at(index).trimmed().toDouble(&ok));
            valuesOk = valuesOk && ok;
        }
        if (!valuesOk) return {false, QStringLiteral("Line %1 contains a non-numeric robot pose value.").arg(lineNumber)};

        const Vector3 translation = {values[0], values[1], values[2]};
        const Vector4 rotation = tokens.size() == 9
                                     ? Vector4{values[3], values[4], values[5], values[6]}
                                     : Vector4{values[3], values[4], values[5], 0.0};
        const auto normalized = pose::normalize(rotation, translation, inputSpec);
        if (!normalized.success)
            return {false, QStringLiteral("Line %1 robot pose normalization failed: %2")
                               .arg(lineNumber).arg(normalized.error)};

        PoseSample sample;
        sample.id = id;
        sample.imagePath = QFileInfo(tokens.at(1).trimmed()).isAbsolute()
                               ? tokens.at(1).trimmed()
                               : csvDirectory.absoluteFilePath(tokens.at(1).trimmed());
        sample.label = QFileInfo(sample.imagePath).completeBaseName();
        sample.gripperRotation = normalized.rotation;
        sample.gripperTranslation = normalized.translation;
        sample.imageStatus = ImageSampleStatus::NotProcessed;
        samples.append(sample);
    }

    if (samples.isEmpty()) return {false, QStringLiteral("No paired pose-image samples found.")};
    dataset->samples = samples;
    dataset->pointSamples.clear();
    dataset->inputMode = CalibrationInputMode::PosePairs;
    dataset->targetPosesReady = false;
    dataset->results.clear();
    return {true, {}};
}

} // namespace handeye
