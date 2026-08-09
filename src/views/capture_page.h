#pragma once

#include <QWidget>

class QLabel;
class QPushButton;

namespace handeye {

class CapturePage : public QWidget
{
    Q_OBJECT

public:
    explicit CapturePage(QWidget *parent = nullptr);

signals:
    void uploadRobotRequested();
    void uploadImagesRequested();
    void processRequested();
    void viewDataRequested();
    void viewResultsRequested();

public slots:
    void setSummary(int sampleCount, int imageCount, int targetPoseCount);
    void setProcessingResult(int processed, int succeeded);

private:
    QLabel *m_summary = nullptr;
    QLabel *m_processingStatus = nullptr;
};

} // namespace handeye
