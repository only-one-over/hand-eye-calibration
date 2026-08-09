#pragma once

#include "domain/calibration_types.h"

#include <QVector>
#include <QWidget>

class QLabel;
class QLineEdit;
class QTableWidget;

namespace handeye {

class ManualPosePage : public QWidget
{
    Q_OBJECT

public:
    explicit ManualPosePage(QWidget *parent = nullptr);

signals:
    void applyRequested(const QVector<ManualPoseInput> &inputs,
                        const PoseInputSpec &spec, bool calculateAll);
    void goParametersRequested();
    void goDataRequested();
    void goResultsRequested();

public slots:
    void setInputSpec(const PoseInputSpec &spec);

private slots:
    void addInput();
    void updateInput();
    void deleteSelected();
    void clearInputs();
    void applyInputs();
    void applyAndCalculate();
    void onCurrentRowChanged(int row, int column, int previousRow, int previousColumn);

private:
    static bool sameSpec(const PoseInputSpec &left, const PoseInputSpec &right);
    static QStringList rotationLabels(RotationFormat format);
    static QString formatValue(double value);

    bool readInput(ManualPoseInput *input) const;
    void setStatus(const QString &text, bool warning = false);
    void updateSpecWidgets();
    void updateTable();
    void showInput(int row);
    bool canEditWithCurrentSpec();

    PoseInputSpec m_inputSpec;
    PoseInputSpec m_draftSpec;
    PoseInputSpec m_pendingSpec;
    bool m_hasDraftSpec = false;
    bool m_hasPendingSpec = false;
    QVector<ManualPoseInput> m_inputs;

    QLabel *m_specSummary = nullptr;
    QLabel *m_status = nullptr;
    QLineEdit *m_idEdit = nullptr;
    QVector<QLineEdit *> m_tcpTranslationEdits;
    QVector<QLineEdit *> m_tcpRotationEdits;
    QVector<QLineEdit *> m_cameraTranslationEdits;
    QVector<QLineEdit *> m_cameraRotationEdits;
    QVector<QLabel *> m_tcpRotationLabels;
    QVector<QLabel *> m_cameraRotationLabels;
    QTableWidget *m_table = nullptr;
};

} // namespace handeye
