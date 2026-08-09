#pragma once

#include "domain/calibration_types.h"

#include <QWidget>

class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QTableWidget;
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
    void computeFixedTargetRequested(int referenceSampleId);
    void optimizeRequested();
    void reliabilityPipelineRequested(int bootstrapResamples, double confidenceLevel);
    void importValidationRequested();
    void exportRequested(const QString &kind);

public slots:
    void setResults(const QVector<CalibrationResult> &results);
    void setReferenceSampleIds(const QVector<PoseSample> &samples);
    void showReliability(const CalibrationResult &result);
    void showMatrix(const CalibrationResult &result);
    void appendLog(const QString &message);
    void clearResults();
    void showPipelineReport(const ReliabilityPipelineReport &report);

private slots:
    void onResultClicked(const QModelIndex &index);

private:
    ResultTableModel *m_model = nullptr;
    QTableView *m_table = nullptr;
    QLabel *m_reliability = nullptr;
    QPlainTextEdit *m_matrix = nullptr;
    QPlainTextEdit *m_log = nullptr;
    QSpinBox *m_referenceSample = nullptr;
    QSpinBox *m_bootstrapResamples = nullptr;
    QDoubleSpinBox *m_bootstrapConfidence = nullptr;
    QTableWidget *m_poseReportTable = nullptr;
    QTableWidget *m_pipelineTable = nullptr;
    QLabel *m_uncertainty = nullptr;
};

} // namespace handeye
