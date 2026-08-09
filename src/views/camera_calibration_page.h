#pragma once

#include "domain/calibration_types.h"

#include <QPixmap>
#include <QWidget>

class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTableWidget;

namespace handeye {

class CameraCalibrationPage : public QWidget
{
    Q_OBJECT

public:
    explicit CameraCalibrationPage(QWidget *parent = nullptr);

signals:
    void selectImagesRequested();
    void detectRequested();
    void calibrateRequested();
    void applyRequested();
    void clearRequested();
    void goParametersRequested();

public slots:
    void setBoardSpec(const BoardSpec &board);
    void setReport(const CameraCalibrationReport &report);

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void onCurrentRowChanged();

private:
    void updatePreview();
    void updateActions();
    void showSample(int row);
    static QString formatMatrix(const Matrix3 &matrix);

    BoardSpec m_boardSpec;
    CameraCalibrationReport m_report;
    QTableWidget *m_table = nullptr;
    QLabel *m_boardSummary = nullptr;
    QLabel *m_status = nullptr;
    QLabel *m_preview = nullptr;
    QLabel *m_sampleInfo = nullptr;
    QLabel *m_metrics = nullptr;
    QPlainTextEdit *m_matrix = nullptr;
    QPlainTextEdit *m_distortion = nullptr;
    QPixmap m_currentPixmap;
};

} // namespace handeye
