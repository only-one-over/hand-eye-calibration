#pragma once

#include <QWidget>

class QLabel;
class QPushButton;

namespace handeye {

class HomePage : public QWidget
{
    Q_OBJECT

public:
    explicit HomePage(QWidget *parent = nullptr);

signals:
    void startCalibrationRequested();
    void navigateRequested(int pageIndex);

public slots:
    void setSummary(int sampleCount, int imageCount, int targetPoseCount, bool hasResults);
    void setStatus(const QString &status);

private:
    QLabel *m_sampleSummary = nullptr;
    QLabel *m_statusLabel = nullptr;
};

} // namespace handeye
