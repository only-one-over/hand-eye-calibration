#pragma once

#include "domain/calibration_types.h"
#include "models/calibration_session_model.h"

#include <QMainWindow>

class QComboBox;
class QLineEdit;
class QPlainTextEdit;
class QTableView;

namespace handeye {

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void newDataset();
    void generateDemo();
    void importCsv();
    void importJson();
    void exportCsv();
    void exportJson();
    void deleteSelectedSamples();
    void calculateSelected();
    void calculateAll();
    void showSelectedResult(const QModelIndex &index);

private:
    void setupUi();
    void syncSamplesToView();
    void appendLog(const QString &message);
    CalibrationMethod selectedMethod() const;
    void updateMatrixView(const CalibrationResult &result);

    CalibrationDataset m_dataset;
    SampleTableModel *m_sampleModel = nullptr;
    ResultTableModel *m_resultModel = nullptr;
    QTableView *m_sampleTable = nullptr;
    QTableView *m_resultTable = nullptr;
    QComboBox *m_methodCombo = nullptr;
    QComboBox *m_modeCombo = nullptr;
    QLineEdit *m_unitEdit = nullptr;
    QPlainTextEdit *m_matrixView = nullptr;
    QPlainTextEdit *m_logView = nullptr;
};

} // namespace handeye
