#pragma once

#include "domain/calibration_types.h"

#include <QWidget>

class QComboBox;
class QLineEdit;

namespace handeye {

class ParametersPage : public QWidget
{
    Q_OBJECT

public:
    explicit ParametersPage(QWidget *parent = nullptr);

    PoseInputSpec inputSpec() const;
    BoardSpec boardSpec() const;
    CameraIntrinsics cameraIntrinsics() const;
    CalibrationMethod currentMethod() const;
    double passRotationRmseDeg() const;
    double passTranslationRmseM() const;
    QString robotName() const;
    QString cameraName() const;

signals:
    void parametersChanged();

public slots:
    void setRobotCamera(const QString &robot, const QString &camera);
    void setDatasetParameters(const CalibrationDataset &dataset);

private slots:
    void onAdapterChanged(int index);

private:
    void emitParametersChanged();

    QComboBox *m_modeCombo = nullptr;
    QComboBox *m_methodCombo = nullptr;
    QComboBox *m_rotationFormatCombo = nullptr;
    QComboBox *m_angleUnitCombo = nullptr;
    QComboBox *m_lengthUnitCombo = nullptr;
    QComboBox *m_adapterCombo = nullptr;
    QLineEdit *m_robotEdit = nullptr;
    QLineEdit *m_cameraEdit = nullptr;
    QLineEdit *m_boardColumnsEdit = nullptr;
    QLineEdit *m_boardRowsEdit = nullptr;
    QLineEdit *m_squareSizeEdit = nullptr;
    QLineEdit *m_cameraMatrixEdit = nullptr;
    QLineEdit *m_distortionEdit = nullptr;
    QLineEdit *m_passRotationEdit = nullptr;
    QLineEdit *m_passTranslationEdit = nullptr;
};

} // namespace handeye
