#include "io/dataset_io.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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

QJsonObject sampleToJson(const PoseSample &sample)
{
    return {{QStringLiteral("id"), sample.id},
            {QStringLiteral("label"), sample.label},
            {QStringLiteral("gripper_rotation"), vectorToJson(sample.gripperRotation)},
            {QStringLiteral("gripper_translation"), vectorToJson(sample.gripperTranslation)},
            {QStringLiteral("target_rotation"), vectorToJson(sample.targetRotation)},
            {QStringLiteral("target_translation"), vectorToJson(sample.targetTranslation)}};
}

PoseSample sampleFromJson(const QJsonObject &object)
{
    PoseSample sample;
    sample.id = object.value(QStringLiteral("id")).toInt();
    sample.label = object.value(QStringLiteral("label")).toString();
    sample.gripperRotation = vectorFromJson(object.value(QStringLiteral("gripper_rotation")));
    sample.gripperTranslation = vectorFromJson(object.value(QStringLiteral("gripper_translation")));
    sample.targetRotation = vectorFromJson(object.value(QStringLiteral("target_rotation")));
    sample.targetTranslation = vectorFromJson(object.value(QStringLiteral("target_translation")));
    return sample;
}

} // namespace

IoResult writeCsv(const QString &filePath, const CalibrationDataset &dataset)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return {false, file.errorString()};
    QTextStream stream(&file);
    stream << "id,label,gripper_rx,gripper_ry,gripper_rz,gripper_tx,gripper_ty,gripper_tz,"
              "target_rx,target_ry,target_rz,target_tx,target_ty,target_tz\n";
    for (const PoseSample &sample : dataset.samples) {
        stream << sample.id << ',' << sample.label << ',';
        for (const double value : sample.gripperRotation) stream << value << ',';
        for (const double value : sample.gripperTranslation) stream << value << ',';
        for (const double value : sample.targetRotation) stream << value << ',';
        stream << sample.targetTranslation[0] << ',' << sample.targetTranslation[1] << ','
               << sample.targetTranslation[2] << '\n';
    }
    return {true, {}};
}

IoResult readCsv(const QString &filePath, CalibrationDataset *dataset)
{
    if (!dataset)
        return {false, QStringLiteral("目标数据集为空。")};
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return {false, file.errorString()};
    QTextStream stream(&file);
    const QString header = stream.readLine();
    if (!header.startsWith(QStringLiteral("id,label,gripper_rx")))
        return {false, QStringLiteral("CSV 表头不符合要求。")};

    QVector<PoseSample> samples;
    int lineNumber = 1;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) continue;
        const QStringList fields = line.split(',');
        if (fields.size() < 14)
            return {false, QStringLiteral("第 %1 行列数不足，应为 14 列。").arg(lineNumber)};
        bool ok = false;
        PoseSample sample;
        sample.id = fields.at(0).toInt(&ok);
        if (!ok) return {false, QStringLiteral("第 %1 行 ID 无效。").arg(lineNumber)};
        sample.label = fields.at(1);
        double *values[] = {sample.gripperRotation.data(), sample.gripperTranslation.data(),
                            sample.targetRotation.data(), sample.targetTranslation.data()};
        for (int group = 0; group < 4; ++group) {
            for (int item = 0; item < 3; ++item) {
                const int index = 2 + group * 3 + item;
                values[group][item] = fields.at(index).toDouble(&ok);
                if (!ok)
                    return {false, QStringLiteral("第 %1 行第 %2 列不是数字。")
                                .arg(lineNumber).arg(index + 1)};
            }
        }
        samples.append(sample);
    }
    dataset->samples = samples;
    dataset->results.clear();
    return {true, {}};
}

IoResult writeJson(const QString &filePath, const CalibrationDataset &dataset)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly))
        return {false, file.errorString()};
    QJsonArray samples;
    for (const PoseSample &sample : dataset.samples)
        samples.append(sampleToJson(sample));
    QJsonObject root{{QStringLiteral("mode"), dataset.mode == CalibrationMode::EyeInHand ? "eye_in_hand" : "eye_to_hand"},
                     {QStringLiteral("translation_unit"), dataset.translationUnit},
                     {QStringLiteral("created_at"), dataset.createdAt.toString(Qt::ISODate)},
                     {QStringLiteral("samples"), samples}};
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return {true, {}};
}

IoResult readJson(const QString &filePath, CalibrationDataset *dataset)
{
    if (!dataset)
        return {false, QStringLiteral("目标数据集为空。")};
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
        return {false, file.errorString()};
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (document.isNull() || !document.isObject())
        return {false, parseError.errorString()};
    const QJsonObject root = document.object();
    CalibrationDataset parsed;
    parsed.mode = root.value(QStringLiteral("mode")).toString() == QStringLiteral("eye_to_hand")
                      ? CalibrationMode::EyeToHand : CalibrationMode::EyeInHand;
    parsed.translationUnit = root.value(QStringLiteral("translation_unit")).toString(QStringLiteral("m"));
    parsed.createdAt = QDateTime::fromString(root.value(QStringLiteral("created_at")).toString(), Qt::ISODate);
    for (const QJsonValue &value : root.value(QStringLiteral("samples")).toArray())
        parsed.samples.append(sampleFromJson(value.toObject()));
    *dataset = parsed;
    return {true, {}};
}

} // namespace handeye
