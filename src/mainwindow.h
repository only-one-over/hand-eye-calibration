#pragma once

#include "domain/calibration_types.h"
#include "models/calibration_session_model.h"

#include <QMainWindow>

class QComboBox;
class QLineEdit;
class QLabel;
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
    void importValidationCsv();
    void exportCsv();
    void exportJson();
    void exportYaml();
    void exportTxt();
    void exportCpp();
    void exportPython();
    void deleteSelectedSamples();
    void calculateSelected();
    void calculateAll();
    void showSelectedResult(const QModelIndex &index);

private:
    void setupUi();
    void syncSamplesToView();
    void appendLog(const QString &message);
    CalibrationMethod selectedMethod() const;
    void updateInputSpecFromUi();
    CalibrationResult selectedResult() const;
    void applyResidualsToSamples(const ReliabilityReport &report);
    void showReliability(const CalibrationResult &result);
    void updateMatrixView(const CalibrationResult &result);

    CalibrationDataset m_dataset;
    SampleTableModel *m_sampleModel = nullptr;
    ResultTableModel *m_resultModel = nullptr;
    QTableView *m_sampleTable = nullptr;
    QTableView *m_resultTable = nullptr;
    QComboBox *m_methodCombo = nullptr;
    QComboBox *m_modeCombo = nullptr;
    QComboBox *m_rotationFormatCombo = nullptr;
    QComboBox *m_angleUnitCombo = nullptr;
    QComboBox *m_lengthUnitCombo = nullptr;
    QComboBox *m_adapterCombo = nullptr;
    QLineEdit *m_unitEdit = nullptr;
    QLineEdit *m_robotEdit = nullptr;
    QLineEdit *m_cameraEdit = nullptr;
    QLabel *m_reliabilityLabel = nullptr;
    QPlainTextEdit *m_matrixView = nullptr;
    QPlainTextEdit *m_logView = nullptr;
};

} // namespace handeye
