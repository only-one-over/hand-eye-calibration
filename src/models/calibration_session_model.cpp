#include "models/calibration_session_model.h"

#include <QLocale>

namespace handeye {

SampleTableModel::SampleTableModel(QObject *parent) : QAbstractTableModel(parent) {}

int SampleTableModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : m_samples.size();
}

int SampleTableModel::columnCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : 14;
}

QVariant SampleTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_samples.size() || role != Qt::DisplayRole)
        return {};
    const PoseSample &sample = m_samples.at(index.row());
    if (index.column() == 0) return sample.id;
    if (index.column() == 1) return sample.label;
    const double *values[] = {sample.gripperRotation.data(), sample.gripperTranslation.data(),
                              sample.targetRotation.data(), sample.targetTranslation.data()};
    const int valueIndex = index.column() - 2;
    return valueIndex >= 0 && valueIndex < 12
               ? QLocale().toString(values[valueIndex / 3][valueIndex % 3], 'f', 6) : QVariant{};
}

QVariant SampleTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole) return {};
    if (orientation == Qt::Vertical) return section + 1;
    static const QStringList headers = {QStringLiteral("ID"), QStringLiteral("标签"),
        QStringLiteral("G Rx"), QStringLiteral("G Ry"), QStringLiteral("G Rz"),
        QStringLiteral("G Tx"), QStringLiteral("G Ty"), QStringLiteral("G Tz"),
        QStringLiteral("T Rx"), QStringLiteral("T Ry"), QStringLiteral("T Rz"),
        QStringLiteral("T Tx"), QStringLiteral("T Ty"), QStringLiteral("T Tz")};
    return headers.value(section);
}

void SampleTableModel::setSamples(const QVector<PoseSample> &samples)
{
    beginResetModel();
    m_samples = samples;
    endResetModel();
}

void SampleTableModel::clear()
{
    setSamples({});
}

QVector<int> SampleTableModel::idsAt(const QModelIndexList &indexes) const
{
    QSet<int> ids;
    for (const QModelIndex &index : indexes)
        if (index.isValid() && index.row() < m_samples.size()) ids.insert(m_samples.at(index.row()).id);
    return ids.values().toVector();
}

ResultTableModel::ResultTableModel(QObject *parent) : QAbstractTableModel(parent) {}
int ResultTableModel::rowCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : m_results.size(); }
int ResultTableModel::columnCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : 5; }

QVariant ResultTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_results.size() || role != Qt::DisplayRole) return {};
    const CalibrationResult &result = m_results.at(index.row());
    switch (index.column()) {
    case 0: return methodName(result.method);
    case 1: return result.success ? QStringLiteral("成功") : QStringLiteral("失败");
    case 2: return QLocale().toString(result.rotationErrorDeg, 'f', 4);
    case 3: return QLocale().toString(result.translationError, 'f', 6);
    case 4: return result.message;
    }
    return {};
}

QVariant ResultTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole) return {};
    if (orientation == Qt::Vertical) return section + 1;
    return QStringList{QStringLiteral("算法"), QStringLiteral("状态"), QStringLiteral("旋转误差(°)"),
                       QStringLiteral("平移误差"), QStringLiteral("消息")}.value(section);
}

void ResultTableModel::setResults(const QVector<CalibrationResult> &results)
{
    beginResetModel(); m_results = results; endResetModel();
}

CalibrationResult ResultTableModel::resultAt(int row) const
{
    return row >= 0 && row < m_results.size() ? m_results.at(row) : CalibrationResult{};
}

} // namespace handeye
