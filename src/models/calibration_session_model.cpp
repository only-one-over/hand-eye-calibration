#include "models/calibration_session_model.h"

#include <QBrush>
#include <QColor>
#include <QLocale>

namespace handeye {

SampleTableModel::SampleTableModel(QObject *parent) : QAbstractTableModel(parent) {}

int SampleTableModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : m_samples.size();
}

int SampleTableModel::columnCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : 21;
}

QVariant SampleTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_samples.size())
        return {};
    const PoseSample &sample = m_samples.at(index.row());
    if (role == Qt::ForegroundRole && sample.outlier)
        return QBrush(QColor("#E8463A"));
    if (role == Qt::BackgroundRole && sample.outlier)
        return QBrush(QColor("#FCEBEA"));
    if (role != Qt::DisplayRole)
        return {};
    if (index.column() == 0) return sample.id;
    if (index.column() == 1) return sample.label;
    const double *values[] = {sample.gripperRotation.data(), sample.gripperTranslation.data(),
                              sample.targetRotation.data(), sample.targetTranslation.data()};
    const int valueIndex = index.column() - 2;
    if (valueIndex >= 0 && valueIndex < 12)
        return QLocale().toString(values[valueIndex / 3][valueIndex % 3], 'f', 6);
    if (index.column() == 14) return QLocale().toString(sample.rotationResidualDeg, 'f', 5);
    if (index.column() == 15) return QLocale().toString(sample.translationResidualM, 'f', 7);
    if (index.column() == 17) return sample.imagePath;
    if (index.column() == 18) return imageSampleStatusName(sample.imageStatus);
    if (index.column() == 19) return sample.detectedCornerCount;
    if (index.column() == 20) return QLocale().toString(sample.pnpReprojectionRmsePx, 'f', 3);
    if (index.column() == 16) return sample.outlier ? QStringLiteral("异常") : QStringLiteral("正常");
    return {};
}

QVariant SampleTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole) return {};
    if (orientation == Qt::Vertical) return section + 1;
    static const QStringList headers = {QStringLiteral("ID"), QStringLiteral("标签"),
        QStringLiteral("G Rx"), QStringLiteral("G Ry"), QStringLiteral("G Rz"),
        QStringLiteral("G Tx"), QStringLiteral("G Ty"), QStringLiteral("G Tz"),
        QStringLiteral("T Rx"), QStringLiteral("T Ry"), QStringLiteral("T Rz"),
        QStringLiteral("T Tx"), QStringLiteral("T Ty"), QStringLiteral("T Tz"),
        QStringLiteral("旋转残差(°)"), QStringLiteral("平移残差(m)"), QStringLiteral("样本状态")};
    if (section == 17) return QStringLiteral("图片路径");
    if (section == 18) return QStringLiteral("图片状态");
    if (section == 19) return QStringLiteral("角点数");
    if (section == 20) return QStringLiteral("PnP RMSE(px)");
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
int ResultTableModel::columnCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : 11; }

QVariant ResultTableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_results.size()) return {};
    const CalibrationResult &result = m_results.at(index.row());
    if (role == Qt::ForegroundRole) {
        if (result.recommended) return QBrush(QColor("#15A877"));
        if (!result.trainingReport.passed) return QBrush(QColor("#E8463A"));
    }
    if (role == Qt::BackgroundRole) {
        if (result.recommended) return QBrush(QColor("#E8F6F1"));
        if (!result.trainingReport.passed && result.success) return QBrush(QColor("#FCEBEA"));
    }
    if (role != Qt::DisplayRole) return {};
    switch (index.column()) {
    case 0: return methodName(result.method);
    case 1: return result.success ? QStringLiteral("成功") : QStringLiteral("失败");
    case 2: return QLocale().toString(result.trainingReport.rotationRmseDeg, 'f', 5);
    case 3: return QLocale().toString(result.trainingReport.translationRmseM, 'f', 7);
    case 4: return QLocale().toString(result.trainingReport.rotationMeanDeg, 'f', 5);
    case 5: return QLocale().toString(result.trainingReport.rotationMaxDeg, 'f', 5);
    case 6: return QLocale().toString(result.trainingReport.translationMeanM, 'f', 7);
    case 7: return QLocale().toString(result.trainingReport.translationMaxM, 'f', 7);
    case 8: return result.trainingReport.passed ? QStringLiteral("通过") : QStringLiteral("未通过");
    case 9: return result.recommended ? QStringLiteral("推荐") : QString{};
    case 10: return result.message;
    }
    return {};
}

QVariant ResultTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole) return {};
    if (orientation == Qt::Vertical) return section + 1;
    return QStringList{QStringLiteral("算法"), QStringLiteral("状态"), QStringLiteral("旋转 RMSE(°)"),
                       QStringLiteral("平移 RMSE(m)"), QStringLiteral("旋转平均(°)"),
                       QStringLiteral("旋转最大(°)"), QStringLiteral("平移平均(m)"),
                       QStringLiteral("平移最大(m)"), QStringLiteral("可靠性"),
                       QStringLiteral("推荐"), QStringLiteral("消息")}.value(section);
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
