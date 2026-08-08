#pragma once

#include "domain/calibration_types.h"

#include <QAbstractTableModel>

namespace handeye {

class SampleTableModel : public QAbstractTableModel
{
public:
    explicit SampleTableModel(QObject *parent = nullptr);
    int rowCount(const QModelIndex &parent = {}) const override;
    int columnCount(const QModelIndex &parent = {}) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    void setSamples(const QVector<PoseSample> &samples);
    void clear();
    QVector<int> idsAt(const QModelIndexList &indexes) const;

private:
    QVector<PoseSample> m_samples;
};

class ResultTableModel : public QAbstractTableModel
{
public:
    explicit ResultTableModel(QObject *parent = nullptr);
    int rowCount(const QModelIndex &parent = {}) const override;
    int columnCount(const QModelIndex &parent = {}) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    void setResults(const QVector<CalibrationResult> &results);
    CalibrationResult resultAt(int row) const;

private:
    QVector<CalibrationResult> m_results;
};

} // namespace handeye
