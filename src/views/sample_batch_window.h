#pragma once

#include "domain/calibration_types.h"

#include <QDialog>

class QLabel;
class QPushButton;
class QTableView;

namespace handeye {

class SampleTableModel;

class SampleBatchWindow : public QDialog
{
    Q_OBJECT

public:
    explicit SampleBatchWindow(QWidget *parent = nullptr);

public slots:
    void setSamples(const QVector<PoseSample> &samples);

signals:
    void deleteRequested(const QVector<int> &ids);

private slots:
    void deleteSelected();

private:
    SampleTableModel *m_model = nullptr;
    QTableView *m_table = nullptr;
    QLabel *m_summary = nullptr;
    QPushButton *m_deleteButton = nullptr;
};

} // namespace handeye
