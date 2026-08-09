#include "views/parameters_page.h"

#include "core/pose_conversion.h"
#include "io/pose_adapter.h"

#include <QComboBox>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QRegularExpression>
#include <QStandardItemModel>
#include <QStringList>
#include <QVBoxLayout>

#include <QVector>

namespace handeye {

namespace {

RotationFormat rotationFormatFromIndex(int index)
{
    return static_cast<RotationFormat>(index);
}

AngleUnit angleUnitFromIndex(int index)
{
    return index == 1 ? AngleUnit::Degrees : AngleUnit::Radians;
}

LengthUnit lengthUnitFromIndex(int index)
{
    return index == 1 ? LengthUnit::Millimeters : LengthUnit::Meters;
}

PoseAdapterKind adapterFromIndex(int index)
{
    return static_cast<PoseAdapterKind>(index);
}

QVector<double> parseNumbers(const QString &text)
{
    QVector<double> values;
    const QStringList tokens = text.split(QRegularExpression(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);
    for (const QString &token : tokens) {
        bool ok = false;
        const double value = token.toDouble(&ok);
        if (!ok) return {};
        values.append(value);
    }
    return values;
}

} // namespace

ParametersPage::ParametersPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(24, 24, 24, 24);
    layout->setSpacing(14);

    auto *title = new QLabel(QStringLiteral("标定参数"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *basicGroup = new QGroupBox(QStringLiteral("输入与算法"), this);
    auto *basicForm = new QFormLayout(basicGroup);
    m_modeCombo = new QComboBox(basicGroup);
    m_modeCombo->addItem(QStringLiteral("Eye-In-Hand（眼在手）"));
    m_modeCombo->addItem(QStringLiteral("Eye-To-Hand（暂未启用）"));
    if (auto *model = qobject_cast<QStandardItemModel *>(m_modeCombo->model())) model->item(1)->setEnabled(false);
    m_methodCombo = new QComboBox(basicGroup);
    for (CalibrationMethod method : allMethods()) m_methodCombo->addItem(methodName(method));
    m_adapterCombo = new QComboBox(basicGroup);
    for (int index = 0; index < 4; ++index) m_adapterCombo->addItem(adapterName(adapterFromIndex(index)));
    m_rotationFormatCombo = new QComboBox(basicGroup);
    m_rotationFormatCombo->addItems({QStringLiteral("Rodrigues"), QStringLiteral("Euler XYZ"),
                                     QStringLiteral("RPY (Z-Y-X)"), QStringLiteral("Quaternion (w,x,y,z)")});
    m_angleUnitCombo = new QComboBox(basicGroup);
    m_angleUnitCombo->addItems({QStringLiteral("弧度 rad"), QStringLiteral("角度 degree")});
    m_lengthUnitCombo = new QComboBox(basicGroup);
    m_lengthUnitCombo->addItems({QStringLiteral("米 m"), QStringLiteral("毫米 mm")});
    m_robotEdit = new QLineEdit(QStringLiteral("未指定机器人"), basicGroup);
    m_cameraEdit = new QLineEdit(QStringLiteral("未指定相机"), basicGroup);
    basicForm->addRow(QStringLiteral("标定模式"), m_modeCombo);
    basicForm->addRow(QStringLiteral("当前算法"), m_methodCombo);
    basicForm->addRow(QStringLiteral("Pose Adapter"), m_adapterCombo);
    basicForm->addRow(QStringLiteral("输入旋转"), m_rotationFormatCombo);
    basicForm->addRow(QStringLiteral("角度单位"), m_angleUnitCombo);
    basicForm->addRow(QStringLiteral("平移单位"), m_lengthUnitCombo);
    basicForm->addRow(QStringLiteral("机器人"), m_robotEdit);
    basicForm->addRow(QStringLiteral("相机"), m_cameraEdit);
    layout->addWidget(basicGroup);

    auto *boardGroup = new QGroupBox(QStringLiteral("棋盘格与相机"), this);
    auto *boardForm = new QFormLayout(boardGroup);
    m_boardColumnsEdit = new QLineEdit(QStringLiteral("9"), boardGroup);
    m_boardRowsEdit = new QLineEdit(QStringLiteral("6"), boardGroup);
    m_squareSizeEdit = new QLineEdit(QStringLiteral("25"), boardGroup);
    m_cameraMatrixEdit = new QLineEdit(QStringLiteral("1,0,0,0,1,0,0,0,1"), boardGroup);
    m_distortionEdit = new QLineEdit(QStringLiteral("0,0,0,0,0"), boardGroup);
    boardForm->addRow(QStringLiteral("内角点列数"), m_boardColumnsEdit);
    boardForm->addRow(QStringLiteral("内角点行数"), m_boardRowsEdit);
    boardForm->addRow(QStringLiteral("方格尺寸（mm）"), m_squareSizeEdit);
    boardForm->addRow(QStringLiteral("相机矩阵（9个数）"), m_cameraMatrixEdit);
    boardForm->addRow(QStringLiteral("畸变参数（k1,k2,p1,p2,k3）"), m_distortionEdit);
    layout->addWidget(boardGroup);

    auto *thresholdGroup = new QGroupBox(QStringLiteral("可靠性阈值"), this);
    auto *thresholdForm = new QFormLayout(thresholdGroup);
    m_passRotationEdit = new QLineEdit(QStringLiteral("0.5"), thresholdGroup);
    m_passTranslationEdit = new QLineEdit(QStringLiteral("0.001"), thresholdGroup);
    thresholdForm->addRow(QStringLiteral("旋转 RMSE 上限（degree）"), m_passRotationEdit);
    thresholdForm->addRow(QStringLiteral("平移 RMSE 上限（m）"), m_passTranslationEdit);
    layout->addWidget(thresholdGroup);
    layout->addStretch();

    connect(m_adapterCombo, &QComboBox::currentIndexChanged, this, &ParametersPage::onAdapterChanged);
    const auto connectCombo = [this](QComboBox *combo) {
        connect(combo, &QComboBox::currentIndexChanged, this, &ParametersPage::emitParametersChanged);
    };
    connectCombo(m_modeCombo);
    connectCombo(m_methodCombo);
    connectCombo(m_rotationFormatCombo);
    connectCombo(m_angleUnitCombo);
    connectCombo(m_lengthUnitCombo);
    connectCombo(m_adapterCombo);
    const auto connectEdit = [this](QLineEdit *edit) {
        connect(edit, &QLineEdit::editingFinished, this, &ParametersPage::emitParametersChanged);
    };
    for (QLineEdit *edit : {m_robotEdit, m_cameraEdit, m_boardColumnsEdit, m_boardRowsEdit,
                            m_squareSizeEdit, m_cameraMatrixEdit, m_distortionEdit,
                            m_passRotationEdit, m_passTranslationEdit})
        connectEdit(edit);
}

PoseInputSpec ParametersPage::inputSpec() const
{
    PoseInputSpec spec;
    spec.adapter = adapterFromIndex(m_adapterCombo->currentIndex());
    spec.rotationFormat = rotationFormatFromIndex(m_rotationFormatCombo->currentIndex());
    spec.angleUnit = angleUnitFromIndex(m_angleUnitCombo->currentIndex());
    spec.lengthUnit = lengthUnitFromIndex(m_lengthUnitCombo->currentIndex());
    spec.direction = PoseDirection::GripperToBase;
    return spec;
}

BoardSpec ParametersPage::boardSpec() const
{
    BoardSpec board;
    bool okColumns = false;
    bool okRows = false;
    bool okSquare = false;
    board.innerCornersX = m_boardColumnsEdit->text().trimmed().toInt(&okColumns);
    board.innerCornersY = m_boardRowsEdit->text().trimmed().toInt(&okRows);
    board.squareSizeM = m_squareSizeEdit->text().trimmed().toDouble(&okSquare) / 1000.0;
    if (!okColumns || !okRows || !okSquare) board.squareSizeM = 0.0;
    return board;
}

CameraIntrinsics ParametersPage::cameraIntrinsics() const
{
    const QVector<double> cameraValues = parseNumbers(m_cameraMatrixEdit->text());
    const QVector<double> distortionValues = parseNumbers(m_distortionEdit->text());
    CameraIntrinsics intrinsics;
    intrinsics.valid = cameraValues.size() == 9 && distortionValues.size() == 5;
    if (!intrinsics.valid) return intrinsics;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            intrinsics.cameraMatrix[row][col] = cameraValues[row * 3 + col];
    for (int index = 0; index < 5; ++index) intrinsics.distortionCoeffs[index] = distortionValues[index];
    intrinsics.source = QStringLiteral("参数页");
    return intrinsics;
}

CalibrationMethod ParametersPage::currentMethod() const
{
    return allMethods().value(m_methodCombo->currentIndex(), CalibrationMethod::Tsai);
}

double ParametersPage::passRotationRmseDeg() const
{
    return m_passRotationEdit->text().trimmed().toDouble();
}

double ParametersPage::passTranslationRmseM() const
{
    return m_passTranslationEdit->text().trimmed().toDouble();
}

QString ParametersPage::robotName() const { return m_robotEdit->text().trimmed(); }
QString ParametersPage::cameraName() const { return m_cameraEdit->text().trimmed(); }

void ParametersPage::setRobotCamera(const QString &robot, const QString &camera)
{
    m_robotEdit->setText(robot);
    m_cameraEdit->setText(camera);
}

void ParametersPage::setDatasetParameters(const CalibrationDataset &dataset)
{
    const QList<QObject *> controls = {m_modeCombo, m_methodCombo, m_adapterCombo,
                                       m_rotationFormatCombo, m_angleUnitCombo, m_lengthUnitCombo,
                                       m_robotEdit, m_cameraEdit, m_boardColumnsEdit, m_boardRowsEdit,
                                       m_squareSizeEdit, m_cameraMatrixEdit, m_distortionEdit,
                                       m_passRotationEdit, m_passTranslationEdit};
    for (QObject *control : controls) control->blockSignals(true);

    m_modeCombo->setCurrentIndex(dataset.mode == CalibrationMode::EyeToHand ? 1 : 0);
    m_adapterCombo->setCurrentIndex(static_cast<int>(dataset.inputSpec.adapter));
    m_rotationFormatCombo->setCurrentIndex(static_cast<int>(dataset.inputSpec.rotationFormat));
    m_angleUnitCombo->setCurrentIndex(dataset.inputSpec.angleUnit == AngleUnit::Degrees ? 1 : 0);
    m_lengthUnitCombo->setCurrentIndex(dataset.inputSpec.lengthUnit == LengthUnit::Millimeters ? 1 : 0);
    m_robotEdit->setText(dataset.robotName);
    m_cameraEdit->setText(dataset.cameraName);
    m_boardColumnsEdit->setText(QString::number(dataset.boardSpec.innerCornersX));
    m_boardRowsEdit->setText(QString::number(dataset.boardSpec.innerCornersY));
    m_squareSizeEdit->setText(QString::number(dataset.boardSpec.squareSizeM * 1000.0, 'g', 12));

    QStringList cameraValues;
    for (const auto &row : dataset.cameraIntrinsics.cameraMatrix)
        for (double value : row) cameraValues.append(QString::number(value, 'g', 12));
    m_cameraMatrixEdit->setText(cameraValues.join(','));
    QStringList distortionValues;
    for (double value : dataset.cameraIntrinsics.distortionCoeffs)
        distortionValues.append(QString::number(value, 'g', 12));
    m_distortionEdit->setText(distortionValues.join(','));
    m_passRotationEdit->setText(QString::number(dataset.passRotationRmseDeg, 'g', 12));
    m_passTranslationEdit->setText(QString::number(dataset.passTranslationRmseM, 'g', 12));

    for (QObject *control : controls) control->blockSignals(false);
}

void ParametersPage::onAdapterChanged(int index)
{
    const PoseInputSpec spec = pose::defaultSpec(adapterFromIndex(index));
    m_rotationFormatCombo->setCurrentIndex(static_cast<int>(spec.rotationFormat));
    m_angleUnitCombo->setCurrentIndex(spec.angleUnit == AngleUnit::Degrees ? 1 : 0);
    m_lengthUnitCombo->setCurrentIndex(spec.lengthUnit == LengthUnit::Millimeters ? 1 : 0);
    emitParametersChanged();
}

void ParametersPage::emitParametersChanged()
{
    emit parametersChanged();
}

} // namespace handeye
