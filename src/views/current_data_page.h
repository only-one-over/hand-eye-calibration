#pragma once

#include "domain/calibration_types.h"

#include <QWidget>
#include <QPixmap>

class QLabel;
class QTableView;

namespace handeye {

class SampleTableModel;

class CurrentDataPage : public QWidget
{
    Q_OBJECT

public:
    explicit CurrentDataPage(QWidget *parent = nullptr);

signals:
    void deleteRequested(const QVector<int> &ids);
    void nextRequested();

public slots:
    void setSamples(const QVector<PoseSample> &samples);
    void setMode(CalibrationMode mode, CalibrationInputMode inputMode);

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void onCurrentRowChanged(const QModelIndex &current, const QModelIndex &previous);
    void deleteSelected();

private:
    void updatePreview();

    SampleTableModel *m_model = nullptr;
    QTableView *m_table = nullptr;
    QLabel *m_imagePreview = nullptr;
    QLabel *m_imageInfo = nullptr;
    QVector<PoseSample> m_samples;
    QPixmap m_currentPixmap;
};

} // namespace handeye
