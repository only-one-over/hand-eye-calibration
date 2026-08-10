#pragma once

#include "core/board_pdf_generator.h"
#include "domain/calibration_types.h"

#include <QWidget>

class QLabel;
class QPushButton;

namespace handeye {

class BoardPdfPage : public QWidget
{
    Q_OBJECT

public:
    explicit BoardPdfPage(QWidget *parent = nullptr);

signals:
    void generateRequested(BoardPdfOutputMode mode);
    void saveAsRequested(BoardPdfOutputMode mode);
    void openBoardPdfRequested(const QString &path);
    void openBoardPdfDirectoryRequested();
    void openDocumentsDirectoryRequested();
    void openDocumentRequested(const QString &fileName);

public slots:
    void setBoardSpec(const BoardSpec &board);
    void setReport(const BoardPdfReport &report);

private:
    void updateBoardSummary();

    BoardSpec m_boardSpec;
    QString m_lastPdfPath;
    QLabel *m_boardSummary = nullptr;
    QLabel *m_outputStatus = nullptr;
    QPushButton *m_openPdfButton = nullptr;
    QPushButton *m_openDirectoryButton = nullptr;
};

} // namespace handeye
