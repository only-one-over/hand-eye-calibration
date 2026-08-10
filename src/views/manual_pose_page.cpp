#include "views/manual_pose_page.h"

#include <QAbstractItemView>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace handeye {

namespace {

QLineEdit *makeEdit(QWidget *parent)
{
    auto *edit = new QLineEdit(parent);
    edit->setPlaceholderText(QStringLiteral("数值"));
    edit->setClearButtonEnabled(true);
    return edit;
}

QStringList rotationLabels(RotationFormat format)
{
    switch (format) {
    case RotationFormat::Rodrigues: return {QStringLiteral("rx"), QStringLiteral("ry"), QStringLiteral("rz")};
    case RotationFormat::EulerXYZ: return {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")};
    case RotationFormat::RPY: return {QStringLiteral("roll"), QStringLiteral("pitch"), QStringLiteral("yaw")};
    case RotationFormat::QuaternionWXYZ:
        return {QStringLiteral("w"), QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z")};
    }
    return {QStringLiteral("r1"), QStringLiteral("r2"), QStringLiteral("r3")};
}

QString valueText(double value)
{
    return QString::number(value, 'g', 12);
}

} // namespace

ManualPosePage::ManualPosePage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(20, 20, 20, 20);
    layout->setSpacing(12);

    auto *title = new QLabel(QStringLiteral("手动输入 TCP 与相机位姿"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *description = new QLabel(this);
    description->setObjectName(QStringLiteral("manualPoseDescription"));
    description->setWordWrap(true);
    layout->addWidget(description);

    m_specSummary = new QLabel(this);
    m_specSummary->setObjectName(QStringLiteral("manualPoseSpecSummary"));
    m_specSummary->setWordWrap(true);
    layout->addWidget(m_specSummary);

    auto *inputGroup = new QGroupBox(QStringLiteral("新增或编辑一组样本"), this);
    auto *inputLayout = new QHBoxLayout(inputGroup);

    auto *tcpGroup = new QGroupBox(QStringLiteral("TCP 末端（gripper→base）"), inputGroup);
    auto *tcpForm = new QFormLayout(tcpGroup);
    m_idEdit = makeEdit(tcpGroup);
    tcpForm->addRow(QStringLiteral("样本 ID"), m_idEdit);
    for (const QString &name : {QStringLiteral("Tx"), QStringLiteral("Ty"), QStringLiteral("Tz")}) {
        auto *edit = makeEdit(tcpGroup);
        m_tcpTranslationEdits.append(edit);
        tcpForm->addRow(name, edit);
    }
    for (int index = 0; index < 4; ++index) {
        auto *label = new QLabel(tcpGroup);
        auto *edit = makeEdit(tcpGroup);
        m_tcpRotationLabels.append(label);
        m_tcpRotationEdits.append(edit);
        tcpForm->addRow(label, edit);
    }
    inputLayout->addWidget(tcpGroup, 1);

    m_pointGroup = new QGroupBox(QStringLiteral("TCP 上特征点（camera 坐标系，仅 XYZ）"), inputGroup);
    auto *pointForm = new QFormLayout(m_pointGroup);
    for (const QString &name : {QStringLiteral("Xc"), QStringLiteral("Yc"), QStringLiteral("Zc")}) {
        auto *edit = makeEdit(m_pointGroup);
        m_cameraPointEdits.append(edit);
        pointForm->addRow(name, edit);
    }
    auto *pointHint = new QLabel(QStringLiteral("Eye-To-Hand 点基：特征点必须刚性安装在 TCP 上，并随 TCP 一起运动。"), m_pointGroup);
    pointHint->setWordWrap(true);
    pointForm->addRow(pointHint);
    inputLayout->addWidget(m_pointGroup, 1);

    m_cameraPoseGroup = new QGroupBox(QStringLiteral("标定板位姿（target→camera）"), inputGroup);
    auto *cameraPoseForm = new QFormLayout(m_cameraPoseGroup);
    for (const QString &name : {QStringLiteral("Tx"), QStringLiteral("Ty"), QStringLiteral("Tz")}) {
        auto *edit = makeEdit(m_cameraPoseGroup);
        m_cameraTranslationEdits.append(edit);
        cameraPoseForm->addRow(name, edit);
    }
    for (int index = 0; index < 4; ++index) {
        auto *label = new QLabel(m_cameraPoseGroup);
        auto *edit = makeEdit(m_cameraPoseGroup);
        m_cameraRotationLabels.append(label);
        m_cameraRotationEdits.append(edit);
        cameraPoseForm->addRow(label, edit);
    }
    auto *poseHint = new QLabel(QStringLiteral("Eye-To-Hand PosePairs：标定板必须刚性安装在 TCP 上。"), m_cameraPoseGroup);
    poseHint->setWordWrap(true);
    cameraPoseForm->addRow(poseHint);
    inputLayout->addWidget(m_cameraPoseGroup, 1);

    auto *buttonLayout = new QVBoxLayout;
    auto *addButton = new QPushButton(QStringLiteral("添加一组"), inputGroup);
    auto *updateButton = new QPushButton(QStringLiteral("更新选中"), inputGroup);
    auto *clearButton = new QPushButton(QStringLiteral("清空输入"), inputGroup);
    addButton->setObjectName(QStringLiteral("addManualPoseButton"));
    updateButton->setObjectName(QStringLiteral("updateManualPoseButton"));
    clearButton->setObjectName(QStringLiteral("clearManualPoseButton"));
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(updateButton);
    buttonLayout->addWidget(clearButton);
    buttonLayout->addStretch();
    inputLayout->addLayout(buttonLayout);
    layout->addWidget(inputGroup);

    m_table = new QTableWidget(this);
    m_table->setObjectName(QStringLiteral("manualPoseTable"));
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    layout->addWidget(m_table, 1);

    auto *actions = new QHBoxLayout;
    auto *deleteButton = new QPushButton(QStringLiteral("删除选中"), this);
    auto *applyButton = new QPushButton(QStringLiteral("应用到当前标定数据"), this);
    auto *calculateButton = new QPushButton(QStringLiteral("应用并计算"), this);
    auto *parametersButton = new QPushButton(QStringLiteral("前往参数页"), this);
    auto *dataButton = new QPushButton(QStringLiteral("查看当前数据"), this);
    auto *resultsButton = new QPushButton(QStringLiteral("查看标定结果"), this);
    applyButton->setObjectName(QStringLiteral("applyManualPoseButton"));
    calculateButton->setObjectName(QStringLiteral("applyAndCalculateManualPoseButton"));
    calculateButton->setProperty("variant", "primary");
    actions->addWidget(deleteButton);
    actions->addStretch();
    actions->addWidget(applyButton);
    actions->addWidget(calculateButton);
    actions->addWidget(parametersButton);
    actions->addWidget(dataButton);
    actions->addWidget(resultsButton);
    layout->addLayout(actions);

    m_status = new QLabel(QStringLiteral("尚未添加手动样本。"), this);
    m_status->setObjectName(QStringLiteral("manualPoseStatus"));
    m_status->setWordWrap(true);
    layout->addWidget(m_status);

    connect(addButton, &QPushButton::clicked, this, &ManualPosePage::addInput);
    connect(updateButton, &QPushButton::clicked, this, &ManualPosePage::updateInput);
    connect(deleteButton, &QPushButton::clicked, this, &ManualPosePage::deleteSelected);
    connect(clearButton, &QPushButton::clicked, this, &ManualPosePage::clearInputs);
    connect(applyButton, &QPushButton::clicked, this, &ManualPosePage::applyInputs);
    connect(calculateButton, &QPushButton::clicked, this, &ManualPosePage::applyAndCalculate);
    connect(parametersButton, &QPushButton::clicked, this, &ManualPosePage::goParametersRequested);
    connect(dataButton, &QPushButton::clicked, this, &ManualPosePage::goDataRequested);
    connect(resultsButton, &QPushButton::clicked, this, &ManualPosePage::goResultsRequested);
    connect(m_table, &QTableWidget::currentCellChanged, this, &ManualPosePage::onCurrentRowChanged);

    updateSpecWidgets();
    updateTable();
}

bool ManualPosePage::sameSpec(const PoseInputSpec &left, const PoseInputSpec &right)
{
    return left.rotationFormat == right.rotationFormat && left.convention == right.convention
           && left.angleUnit == right.angleUnit && left.lengthUnit == right.lengthUnit
           && left.adapter == right.adapter && left.quaternionWFirst == right.quaternionWFirst;
}

void ManualPosePage::setInputSpec(const PoseInputSpec &spec)
{
    if ((!m_inputs.isEmpty() || !m_poseInputs.isEmpty()) && !sameSpec(spec, m_draftSpec)) {
        m_pendingSpec = spec;
        m_hasPendingSpec = true;
        setStatus(QStringLiteral("参数规范已变化；当前草稿仍按旧规范解释，请先清空草稿后再录入。"), true);
        return;
    }
    m_inputSpec = spec;
    m_draftSpec = spec;
    m_hasPendingSpec = false;
    updateSpecWidgets();
}

void ManualPosePage::setCalibrationMode(CalibrationMode mode, CalibrationInputMode inputMode)
{
    if (m_mode == mode && m_inputMode == inputMode) {
        updateSpecWidgets();
        updateTable();
        return;
    }
    if (!m_inputs.isEmpty() || !m_poseInputs.isEmpty()) {
        m_inputs.clear();
        m_poseInputs.clear();
        m_hasDraftSpec = false;
        setStatus(QStringLiteral("标定模式或输入模式变化，旧手动草稿已清空，请重新录入。"), true);
    }
    m_mode = mode;
    m_inputMode = inputMode;
    updateSpecWidgets();
    updateTable();
}

bool ManualPosePage::readPoseInput(ManualPoseInput *input) const
{
    if (!input) return false;
    bool idOk = false;
    input->id = m_idEdit->text().trimmed().toInt(&idOk);
    if (!idOk || input->id <= 0) return false;
    const auto readVector = [](const QVector<QLineEdit *> &edits, auto *output, int count) {
        for (int index = 0; index < count; ++index) {
            bool ok = false;
            const double value = edits.at(index)->text().trimmed().toDouble(&ok);
            if (!ok || !std::isfinite(value)) return false;
            (*output)[index] = value;
        }
        return true;
    };
    const int count = m_inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ ? 4 : 3;
    if (!readVector(m_tcpTranslationEdits, &input->tcpTranslation, 3)
        || !readVector(m_tcpRotationEdits, &input->tcpRotation, count)
        || !readVector(m_cameraTranslationEdits, &input->cameraTranslation, 3)
        || !readVector(m_cameraRotationEdits, &input->cameraRotation, count))
        return false;
    input->label = QStringLiteral("手动 PosePairs 输入 #%1").arg(input->id);
    return true;
}

bool ManualPosePage::readInput(PointSample *input) const
{
    if (!input || m_inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ) return false;
    bool idOk = false;
    input->id = m_idEdit->text().trimmed().toInt(&idOk);
    if (!idOk || input->id <= 0) return false;
    const auto readVector = [](const QVector<QLineEdit *> &edits, auto *output) {
        for (int index = 0; index < 3; ++index) {
            bool ok = false;
            const double value = edits.at(index)->text().trimmed().toDouble(&ok);
            if (!ok || !std::isfinite(value)) return false;
            (*output)[index] = value;
        }
        return true;
    };
    if (!readVector(m_tcpTranslationEdits, &input->gripperTranslation)
        || !readVector(m_tcpRotationEdits, &input->gripperRotation)
        || !readVector(m_cameraPointEdits, &input->cameraPoint))
        return false;
    input->label = QStringLiteral("手动点基输入 #%1").arg(input->id);
    return true;
}

bool ManualPosePage::canEditWithCurrentSpec()
{
    if (m_hasPendingSpec) {
        setStatus(QStringLiteral("当前草稿使用旧输入规范，请先清空草稿，再使用新规范录入。"), true);
        return false;
    }
    if (m_inputMode == CalibrationInputMode::FixedPoint3D
        && m_inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ) {
        setStatus(QStringLiteral("FixedPoint3D 手动输入需要三轴 TCP 旋转；请选择 Rodrigues、Euler XYZ 或 RPY。"), true);
        return false;
    }
    return true;
}

void ManualPosePage::addInput()
{
    if (!canEditWithCurrentSpec()) return;
    if (m_inputMode == CalibrationInputMode::PosePairs) {
        ManualPoseInput input;
        if (!readPoseInput(&input)) {
            setStatus(QStringLiteral("请填写有效的 TCP 与 target→camera 位姿数字。"), true);
            return;
        }
        for (const ManualPoseInput &item : m_poseInputs)
            if (item.id == input.id) {
                setStatus(QStringLiteral("样本 ID 已存在：%1。" ).arg(input.id), true);
                return;
            }
        m_poseInputs.append(input);
        m_hasDraftSpec = true;
    } else {
        PointSample input;
        if (!readInput(&input)) {
            setStatus(QStringLiteral("请填写有效数字；点基模式需要 TCP 三轴旋转和相机 XYZ。"), true);
            return;
        }
        for (const PointSample &item : m_inputs)
            if (item.id == input.id) {
                setStatus(QStringLiteral("样本 ID 已存在：%1。" ).arg(input.id), true);
                return;
            }
        m_inputs.append(input);
        m_hasDraftSpec = true;
    }
    updateTable();
    m_table->selectRow(m_table->rowCount() - 1);
    setStatus(QStringLiteral("已添加样本，当前共 %1 组。" ).arg(m_table->rowCount()));
}

void ManualPosePage::updateInput()
{
    if (!canEditWithCurrentSpec()) return;
    const int row = m_table->currentRow();
    const int count = m_inputMode == CalibrationInputMode::PosePairs ? m_poseInputs.size() : m_inputs.size();
    if (row < 0 || row >= count) {
        setStatus(QStringLiteral("请先选择要更新的样本。"), true);
        return;
    }
    if (m_inputMode == CalibrationInputMode::PosePairs) {
        ManualPoseInput input;
        if (!readPoseInput(&input)) {
            setStatus(QStringLiteral("请填写有效的 TCP 与 target→camera 位姿数字。"), true);
            return;
        }
        for (int index = 0; index < m_poseInputs.size(); ++index)
            if (index != row && m_poseInputs.at(index).id == input.id) {
                setStatus(QStringLiteral("样本 ID 已存在：%1。" ).arg(input.id), true);
                return;
            }
        m_poseInputs[row] = input;
    } else {
        PointSample input;
        if (!readInput(&input)) {
            setStatus(QStringLiteral("请填写有效数字；点基模式需要 TCP 三轴旋转和相机 XYZ。"), true);
            return;
        }
        for (int index = 0; index < m_inputs.size(); ++index)
            if (index != row && m_inputs.at(index).id == input.id) {
                setStatus(QStringLiteral("样本 ID 已存在：%1。" ).arg(input.id), true);
                return;
            }
        m_inputs[row] = input;
    }
    updateTable();
    m_table->selectRow(row);
    setStatus(QStringLiteral("已更新选中样本。"));
}

void ManualPosePage::deleteSelected()
{
    const int row = m_table->currentRow();
    const int count = m_inputMode == CalibrationInputMode::PosePairs ? m_poseInputs.size() : m_inputs.size();
    if (row < 0 || row >= count) {
        setStatus(QStringLiteral("请先选择要删除的样本。"), true);
        return;
    }
    if (m_inputMode == CalibrationInputMode::PosePairs) m_poseInputs.removeAt(row);
    else m_inputs.removeAt(row);
    updateTable();
    if (m_table->rowCount() > 0) m_table->selectRow(std::min(row, m_table->rowCount() - 1));
    setStatus(QStringLiteral("已删除选中样本，当前剩余 %1 组。" ).arg(m_table->rowCount()));
}

void ManualPosePage::clearInputs()
{
    if (!m_inputs.isEmpty() || !m_poseInputs.isEmpty()) {
        if (QMessageBox::question(this, QStringLiteral("清空手动数据"), QStringLiteral("确定清空当前输入草稿吗？"),
                                  QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
            return;
    }
    m_inputs.clear();
    m_poseInputs.clear();
    m_hasDraftSpec = false;
    if (m_hasPendingSpec) {
        m_inputSpec = m_pendingSpec;
        m_draftSpec = m_pendingSpec;
        m_hasPendingSpec = false;
    } else {
        m_draftSpec = m_inputSpec;
    }
    updateSpecWidgets();
    updateTable();
    setStatus(QStringLiteral("已清空手动输入草稿。"));
}

void ManualPosePage::applyInputs()
{
    const int count = m_inputMode == CalibrationInputMode::PosePairs ? m_poseInputs.size() : m_inputs.size();
    const int minimum = m_inputMode == CalibrationInputMode::PosePairs ? 3 : 5;
    if (count < minimum) {
        setStatus(QStringLiteral("当前输入模式至少需要 %1 组样本。" ).arg(minimum), true);
        return;
    }
    if (!canEditWithCurrentSpec()) return;
    if (m_inputMode == CalibrationInputMode::PosePairs) emit applyPoseRequested(m_poseInputs, m_draftSpec, false);
    else emit applyPointRequested(m_inputs, m_draftSpec, false);
}

void ManualPosePage::applyAndCalculate()
{
    const int count = m_inputMode == CalibrationInputMode::PosePairs ? m_poseInputs.size() : m_inputs.size();
    const int minimum = m_inputMode == CalibrationInputMode::PosePairs ? 3 : 5;
    if (count < minimum) {
        setStatus(QStringLiteral("当前输入模式至少需要 %1 组样本。" ).arg(minimum), true);
        return;
    }
    if (!canEditWithCurrentSpec()) return;
    if (m_inputMode == CalibrationInputMode::PosePairs) emit applyPoseRequested(m_poseInputs, m_draftSpec, true);
    else emit applyPointRequested(m_inputs, m_draftSpec, true);
}

void ManualPosePage::onCurrentRowChanged(int row, int, int, int)
{
    showInput(row);
}

void ManualPosePage::setStatus(const QString &text, bool warning)
{
    m_status->setText(text);
    m_status->setStyleSheet(warning ? QStringLiteral("color: #C0392B;") : QString{});
}

void ManualPosePage::updateSpecWidgets()
{
    const PoseInputSpec &active = m_hasDraftSpec ? m_draftSpec : m_inputSpec;
    const QStringList labels = rotationLabels(active.rotationFormat);
    const bool quaternion = active.rotationFormat == RotationFormat::QuaternionWXYZ;
    for (int index = 0; index < m_tcpRotationLabels.size(); ++index) {
        const bool visible = index < labels.size();
        m_tcpRotationLabels.at(index)->setText(index < labels.size() ? labels.at(index) : QStringLiteral("r4"));
        m_tcpRotationLabels.at(index)->setVisible(visible);
        m_tcpRotationEdits.at(index)->setVisible(visible);
        m_cameraRotationLabels.at(index)->setText(index < labels.size() ? labels.at(index) : QStringLiteral("r4"));
        m_cameraRotationLabels.at(index)->setVisible(visible);
        m_cameraRotationEdits.at(index)->setVisible(visible);
    }
    m_pointGroup->setVisible(m_inputMode == CalibrationInputMode::FixedPoint3D);
    m_cameraPoseGroup->setVisible(m_inputMode == CalibrationInputMode::PosePairs);
    if (auto *calculateButton = findChild<QPushButton *>(QStringLiteral("applyAndCalculateManualPoseButton")))
        calculateButton->setText(m_inputMode == CalibrationInputMode::FixedPoint3D
                                     ? QStringLiteral("应用并计算点基")
                                     : m_mode == CalibrationMode::EyeToHand
                                           ? QStringLiteral("应用并计算 Shah / Li")
                                           : QStringLiteral("应用并计算五种算法"));
    const QString modeText = m_mode == CalibrationMode::EyeToHand ? QStringLiteral("Eye-To-Hand（眼在手外）")
                                                                    : QStringLiteral("Eye-In-Hand（眼在手）");
    if (m_inputMode == CalibrationInputMode::FixedPoint3D) {
        m_specSummary->setText(QStringLiteral("%1 | FixedPoint3D：机器人 gripper→base；相机输入 XYZ；内部单位 m/rad。\n旋转格式：%2，角度：%3，长度：%4")
                                   .arg(modeText, rotationFormatName(active.rotationFormat), angleUnitName(active.angleUnit), lengthUnitName(active.lengthUnit)));
        findChild<QLabel *>(QStringLiteral("manualPoseDescription"))->setText(
            QStringLiteral("点基模式不输入相机 rx/ry/rz。Eye-To-Hand 要求相机测得的是随 TCP 运动的同一个刚性特征点。"));
    } else {
        m_specSummary->setText(QStringLiteral("%1 | PosePairs：机器人 gripper→base；相机 target→camera；内部单位 m/rad。\n旋转格式：%2，角度：%3，长度：%4")
                                   .arg(modeText, rotationFormatName(active.rotationFormat), angleUnitName(active.angleUnit), lengthUnitName(active.lengthUnit)));
        findChild<QLabel *>(QStringLiteral("manualPoseDescription"))->setText(
            QStringLiteral("每组 TCP 位姿必须与同一时刻的标定板 target→camera 位姿一一对应。Eye-To-Hand 中标定板应刚性安装在 TCP。"));
    }
    Q_UNUSED(quaternion)
}

void ManualPosePage::updateTable()
{
    m_table->setUpdatesEnabled(false);
    m_table->clearContents();
    if (m_inputMode == CalibrationInputMode::PosePairs) {
        m_table->setColumnCount(9);
        m_table->setHorizontalHeaderLabels({QStringLiteral("ID"), QStringLiteral("TCP Tx"), QStringLiteral("TCP Ty"), QStringLiteral("TCP Tz"),
                                            QStringLiteral("TCP rx"), QStringLiteral("TCP ry"), QStringLiteral("TCP rz"), QStringLiteral("Camera Tx"), QStringLiteral("状态")});
        m_table->setRowCount(m_poseInputs.size());
        for (int row = 0; row < m_poseInputs.size(); ++row) {
            const ManualPoseInput &input = m_poseInputs.at(row);
            m_table->setItem(row, 0, new QTableWidgetItem(QString::number(input.id)));
            for (int index = 0; index < 3; ++index) {
                m_table->setItem(row, index + 1, new QTableWidgetItem(valueText(input.tcpTranslation[index])));
                m_table->setItem(row, index + 4, new QTableWidgetItem(valueText(input.tcpRotation[index])));
            }
            m_table->setItem(row, 7, new QTableWidgetItem(valueText(input.cameraTranslation[0])));
            m_table->setItem(row, 8, new QTableWidgetItem(QStringLiteral("待应用")));
        }
    } else {
        m_table->setColumnCount(11);
        m_table->setHorizontalHeaderLabels({QStringLiteral("ID"), QStringLiteral("TCP Tx"), QStringLiteral("TCP Ty"), QStringLiteral("TCP Tz"),
                                            QStringLiteral("TCP R1"), QStringLiteral("TCP R2"), QStringLiteral("TCP R3"),
                                            QStringLiteral("Camera X"), QStringLiteral("Camera Y"), QStringLiteral("Camera Z"), QStringLiteral("状态")});
        m_table->setRowCount(m_inputs.size());
        for (int row = 0; row < m_inputs.size(); ++row) {
            const PointSample &input = m_inputs.at(row);
            m_table->setItem(row, 0, new QTableWidgetItem(QString::number(input.id)));
            for (int index = 0; index < 3; ++index) {
                m_table->setItem(row, index + 1, new QTableWidgetItem(valueText(input.gripperTranslation[index])));
                m_table->setItem(row, index + 4, new QTableWidgetItem(valueText(input.gripperRotation[index])));
                m_table->setItem(row, index + 7, new QTableWidgetItem(valueText(input.cameraPoint[index])));
            }
            m_table->setItem(row, 10, new QTableWidgetItem(QStringLiteral("待应用")));
        }
    }
    m_table->setUpdatesEnabled(true);
}

void ManualPosePage::showInput(int row)
{
    const int count = m_inputMode == CalibrationInputMode::PosePairs ? m_poseInputs.size() : m_inputs.size();
    if (row < 0 || row >= count) return;
    m_idEdit->setText(QString::number(m_inputMode == CalibrationInputMode::PosePairs ? m_poseInputs.at(row).id : m_inputs.at(row).id));
    if (m_inputMode == CalibrationInputMode::PosePairs) {
        const ManualPoseInput &input = m_poseInputs.at(row);
        for (int index = 0; index < 3; ++index) {
            m_tcpTranslationEdits.at(index)->setText(valueText(input.tcpTranslation[index]));
            m_cameraTranslationEdits.at(index)->setText(valueText(input.cameraTranslation[index]));
        }
        for (int index = 0; index < 4; ++index) {
            m_tcpRotationEdits.at(index)->setText(valueText(input.tcpRotation[index]));
            m_cameraRotationEdits.at(index)->setText(valueText(input.cameraRotation[index]));
        }
    } else {
        const PointSample &input = m_inputs.at(row);
        for (int index = 0; index < 3; ++index) {
            m_tcpTranslationEdits.at(index)->setText(valueText(input.gripperTranslation[index]));
            m_tcpRotationEdits.at(index)->setText(valueText(input.gripperRotation[index]));
            m_cameraPointEdits.at(index)->setText(valueText(input.cameraPoint[index]));
        }
    }
}

} // namespace handeye
