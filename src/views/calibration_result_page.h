#pragma once

#include "domain/calibration_types.h"

#include <QWidget>

class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTableView;

namespace handeye {

class ResultTableModel;

class CalibrationResultPage : public QWidget
{
    Q_OBJECT

public:
    explicit CalibrationResultPage(QWidget *parent = nullptr);
    CalibrationResult selectedResult() const;

signals:
    void calculateSelectedRequested();
    void calculateAllRequested();
    void importValidationRequested();
    void exportRequested(const QString &kind);

public slots:
    void setResults(const QVector<CalibrationResult> &results);
    void showReliability(const CalibrationResult &result);
    void showMatrix(const CalibrationResult &result);
    void appendLog(const QString &message);
    void clearResults();

private slots:
    void onResultClicked(const QModelIndex &index);

private:
    ResultTableModel *m_model = nullptr;
    QTableView *m_table = nullptr;
    QLabel *m_reliability = nullptr;
    QPlainTextEdit *m_matrix = nullptr;
    QPlainTextEdit *m_log = nullptr;
};

} // namespace handeye
