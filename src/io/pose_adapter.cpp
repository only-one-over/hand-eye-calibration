#include "io/pose_adapter.h"

#include "core/pose_conversion.h"

#include <QRegularExpression>

namespace handeye {

QString adapterName(PoseAdapterKind adapter)
{
    switch (adapter) {
    case PoseAdapterKind::Generic: return QStringLiteral("Generic 6D");
    case PoseAdapterKind::UniversalRobots: return QStringLiteral("Universal Robots [x,y,z,rx,ry,rz]");
    case PoseAdapterKind::Kuka: return QStringLiteral("KUKA XYZABC (mm/deg)");
    case PoseAdapterKind::Fanuc: return QStringLiteral("FANUC XYZWPR (mm/deg)");
    }
    return QStringLiteral("Unknown");
}

AdapterResult parseRobotPoseLine(const QString &line, PoseAdapterKind adapter, int lineNumber)
{
    const QStringList tokens = line.trimmed().split(QRegularExpression(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);
    AdapterResult result;
    if (tokens.size() != 6) {
        result.error = QStringLiteral("第 %1 行应包含 6 个位姿数值。").arg(lineNumber);
        return result;
    }
    bool ok = false;
    QVector<double> values;
    for (const QString &token : tokens) {
        const double value = token.toDouble(&ok);
        if (!ok) {
            result.error = QStringLiteral("第 %1 行包含非数字字段：%2").arg(lineNumber).arg(token);
            return result;
        }
        values.append(value);
    }
    const PoseInputSpec spec = pose::defaultSpec(adapter);
    const auto normalized = pose::normalize({values[3], values[4], values[5], 0.0},
                                            {values[0], values[1], values[2]}, spec);
    result.success = normalized.success;
    result.rotation = normalized.rotation;
    result.translation = normalized.translation;
    result.error = normalized.error;
    return result;
}

} // namespace handeye
