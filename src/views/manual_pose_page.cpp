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

QLineEdit *makeEdit(QWidget *parent, const QString &placeholder = {})
{
    auto *edit = new QLineEdit(parent);
    edit->setPlaceholderText(placeholder);
    edit->setClearButtonEnabled(true);
    return edit;
}

QString poseSummary(const PoseInputSpec &spec)
{
    return QStringLiteral("TCP：%1 | 相机：%2 | 旋转：%3 | 角度：%4 | 平移：%5")
        .arg(directionName(PoseDirection::GripperToBase), directionName(PoseDirection::TargetToCamera),
             rotationFormatName(spec.rotationFormat), angleUnitName(spec.angleUnit),
             lengthUnitName(spec.lengthUnit));
}

} // namespace

ManualPosePage::ManualPosePage(QWidget *parent) : QWidget(parent)
{
    m_inputSpec = PoseInputSpec{};
    m_draftSpec = m_inputSpec;

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(20, 20, 20, 20);
    layout->setSpacing(12);

    auto *title = new QLabel(QStringLiteral("手动输入 TCP 与相机位姿"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *description = new QLabel(
        QStringLiteral("每一行必须来自同一次采样：输入机器人 TCP 的 gripper→base 位姿，以及相机测得的标定板 target→camera 位姿。应用后将替换当前训练数据。"),
        this);
    description->setWordWrap(true);
    layout->addWidget(description);

    m_specSummary = new QLabel(this);
    m_specSummary->setObjectName(QStringLiteral("manualPoseSpecSummary"));
    m_specSummary->setWordWrap(true);
    layout->addWidget(m_specSummary);

    auto *inputGroup = new QGroupBox(QStringLiteral("新增或编辑一组位姿"), this);
    auto *inputLayout = new QHBoxLayout(inputGroup);

    auto *tcpGroup = new QGroupBox(QStringLiteral("TCP 末端数据（gripper→base）"), inputGroup);
    auto *tcpForm = new QFormLayout(tcpGroup);
    m_idEdit = makeEdit(tcpGroup, QStringLiteral("例如 1"));
    tcpForm->addRow(QStringLiteral("样本 ID"), m_idEdit);
    for (const QString &name : {QStringLiteral("Tx"), QStringLiteral("Ty"), QStringLiteral("Tz")}) {
        auto *edit = makeEdit(tcpGroup, QStringLiteral("数值"));
        m_tcpTranslationEdits.append(edit);
        tcpForm->addRow(name, edit);
    }
    const QStringList defaultRotationLabels = rotationLabels(m_inputSpec.rotationFormat);
    for (int index = 0; index < 3; ++index) {
        const QString name = defaultRotationLabels.at(index);
        auto *edit = makeEdit(tcpGroup, QStringLiteral("数值"));
        auto *label = new QLabel(name, tcpGroup);
        m_tcpRotationEdits.append(edit);
        m_tcpRotationLabels.append(label);
        tcpForm->addRow(label, edit);
    }
    auto *tcpFourthEdit = makeEdit(tcpGroup, QStringLiteral("Quaternion 第 4 个分量"));
    auto *tcpFourthLabel = new QLabel(defaultRotationLabels.size() > 3
                                          ? defaultRotationLabels.at(3)
                                          : QStringLiteral("第4分量"),
                                      tcpGroup);
    m_tcpRotationEdits.append(tcpFourthEdit);
    m_tcpRotationLabels.append(tcpFourthLabel);
    tcpForm->addRow(tcpFourthLabel, tcpFourthEdit);
    inputLayout->addWidget(tcpGroup, 1);

    auto *cameraGroup = new QGroupBox(QStringLiteral("相机数据（target→camera）"), inputGroup);
    auto *cameraForm = new QFormLayout(cameraGroup);
    for (const QString &name : {QStringLiteral("Tx"), QStringLiteral("Ty"), QStringLiteral("Tz")}) {
        auto *edit = makeEdit(cameraGroup, QStringLiteral("数值"));
        m_cameraTranslationEdits.append(edit);
        cameraForm->addRow(name, edit);
    }
    for (int index = 0; index < 3; ++index) {
        const QString name = defaultRotationLabels.at(index);
        auto *edit = makeEdit(cameraGroup, QStringLiteral("数值"));
        auto *label = new QLabel(name, cameraGroup);
        m_cameraRotationEdits.append(edit);
        m_cameraRotationLabels.append(label);
        cameraForm->addRow(label, edit);
    }
    auto *cameraFourthEdit = makeEdit(cameraGroup, QStringLiteral("Quaternion 第 4 个分量"));
    auto *cameraFourthLabel = new QLabel(defaultRotationLabels.size() > 3
                                             ? defaultRotationLabels.at(3)
                                             : QStringLiteral("第4分量"),
                                         cameraGroup);
    m_cameraRotationEdits.append(cameraFourthEdit);
    m_cameraRotationLabels.append(cameraFourthLabel);
    cameraForm->addRow(cameraFourthLabel, cameraFourthEdit);
    inputLayout->addWidget(cameraGroup, 1);

    auto *buttonLayout = new QVBoxLayout;
    auto *addButton = new QPushButton(QStringLiteral("添加一组"), inputGroup);
    addButton->setObjectName(QStringLiteral("addManualPoseButton"));
    auto *updateButton = new QPushButton(QStringLiteral("更新选中"), inputGroup);
    updateButton->setObjectName(QStringLiteral("updateManualPoseButton"));
    auto *clearButton = new QPushButton(QStringLiteral("清空输入"), inputGroup);
    clearButton->setObjectName(QStringLiteral("clearManualPoseButton"));
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(updateButton);
    buttonLayout->addWidget(clearButton);
    buttonLayout->addStretch();
    inputLayout->addLayout(buttonLayout);
    layout->addWidget(inputGroup);

    m_table = new QTableWidget(this);
    m_table->setObjectName(QStringLiteral("manualPoseTable"));
    m_table->setColumnCount(16);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    layout->addWidget(m_table, 1);

    auto *actionLayout = new QHBoxLayout;
    auto *deleteButton = new QPushButton(QStringLiteral("删除选中"), this);
    deleteButton->setObjectName(QStringLiteral("deleteManualPoseButton"));
    auto *applyButton = new QPushButton(QStringLiteral("应用到当前标定数据"), this);
    applyButton->setObjectName(QStringLiteral("applyManualPoseButton"));
    applyButton->setProperty("variant", "primary");
    auto *calculateButton = new QPushButton(QStringLiteral("应用并计算五种算法"), this);
    calculateButton->setObjectName(QStringLiteral("applyAndCalculateManualPoseButton"));
    calculateButton->setProperty("variant", "primary");
    auto *parametersButton = new QPushButton(QStringLiteral("前往参数页"), this);
    auto *dataButton = new QPushButton(QStringLiteral("查看当前数据"), this);
    auto *resultsButton = new QPushButton(QStringLiteral("查看标定结果"), this);
    actionLayout->addWidget(deleteButton);
    actionLayout->addStretch();
    actionLayout->addWidget(applyButton);
    actionLayout->addWidget(calculateButton);
    actionLayout->addWidget(parametersButton);
    actionLayout->addWidget(dataButton);
    actionLayout->addWidget(resultsButton);
    layout->addLayout(actionLayout);

    m_status = new QLabel(QStringLiteral("尚未添加手动位姿。"), this);
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
    return left.rotationFormat == right.rotationFormat && left.angleUnit == right.angleUnit
           && left.lengthUnit == right.lengthUnit && left.adapter == right.adapter
           && left.quaternionWFirst == right.quaternionWFirst;
}

QStringList ManualPosePage::rotationLabels(RotationFormat format)
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

QString ManualPosePage::formatValue(double value)
{
    return QString::number(value, 'g', 12);
}

void ManualPosePage::setInputSpec(const PoseInputSpec &spec)
{
    if (!m_inputs.isEmpty() && !sameSpec(spec, m_draftSpec)) {
        m_pendingSpec = spec;
        m_hasPendingSpec = true;
        setStatus(QStringLiteral("参数页的输入规范已变化。当前草稿仍按“%1”解释，请清空草稿后再使用新规范。")
                      .arg(poseSummary(m_draftSpec)), true);
        return;
    }
    m_inputSpec = spec;
    m_draftSpec = spec;
    m_hasPendingSpec = false;
    updateSpecWidgets();
}

bool ManualPosePage::readInput(ManualPoseInput *input) const
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
    const int rotationCount = m_inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ ? 4 : 3;
    if (!readVector(m_tcpTranslationEdits, &input->tcpTranslation, 3)
        || !readVector(m_tcpRotationEdits, &input->tcpRotation, rotationCount)
        || !readVector(m_cameraTranslationEdits, &input->cameraTranslation, 3)
        || !readVector(m_cameraRotationEdits, &input->cameraRotation, rotationCount))
        return false;
    if (m_inputSpec.rotationFormat != RotationFormat::QuaternionWXYZ) {
        input->tcpRotation[3] = 0.0;
        input->cameraRotation[3] = 0.0;
    }
    input->label = QStringLiteral("手动输入 #%1").arg(input->id);
    return true;
}

void ManualPosePage::addInput()
{
    if (!canEditWithCurrentSpec()) return;
    ManualPoseInput input;
    if (!readInput(&input)) {
        setStatus(QStringLiteral("请输入完整且有效的数字；ID 必须为正整数。"), true);
        return;
    }
    for (const ManualPoseInput &existing : m_inputs) {
        if (existing.id == input.id) {
            setStatus(QStringLiteral("样本 ID 已存在：%1，请使用更新选中或修改 ID。" ).arg(input.id), true);
            return;
        }
    }
    if (!m_hasDraftSpec) {
        m_draftSpec = m_inputSpec;
        m_hasDraftSpec = true;
    }
    m_inputs.append(input);
    updateTable();
    m_table->selectRow(m_inputs.size() - 1);
    setStatus(QStringLiteral("已添加第 %1 组手动位姿。当前共 %2 组。")
                  .arg(input.id).arg(m_inputs.size()));
}

void ManualPosePage::updateInput()
{
    if (!canEditWithCurrentSpec()) return;
    const int row = m_table->currentRow();
    if (row < 0 || row >= m_inputs.size()) {
        setStatus(QStringLiteral("请先选择要更新的样本。"), true);
        return;
    }
    ManualPoseInput input;
    if (!readInput(&input)) {
        setStatus(QStringLiteral("请输入完整且有效的数字；ID 必须为正整数。"), true);
        return;
    }
    for (int index = 0; index < m_inputs.size(); ++index) {
        if (index != row && m_inputs.at(index).id == input.id) {
            setStatus(QStringLiteral("样本 ID 已存在：%1。" ).arg(input.id), true);
            return;
        }
    }
    m_inputs[row] = input;
    updateTable();
    m_table->selectRow(row);
    setStatus(QStringLiteral("已更新第 %1 组手动位姿。" ).arg(input.id));
}

void ManualPosePage::deleteSelected()
{
    const int row = m_table->currentRow();
    if (row < 0 || row >= m_inputs.size()) {
        setStatus(QStringLiteral("请先选择要删除的样本。"), true);
        return;
    }
    m_inputs.removeAt(row);
    updateTable();
    if (!m_inputs.isEmpty())
        m_table->selectRow(std::min(row, static_cast<int>(m_inputs.size()) - 1));
    setStatus(QStringLiteral("已删除选中样本，当前剩余 %1 组。" ).arg(m_inputs.size()));
}

void ManualPosePage::clearInputs()
{
    if (!m_inputs.isEmpty()) {
        const auto answer = QMessageBox::question(this, QStringLiteral("清空手动数据"),
                                                  QStringLiteral("确定清空当前手动输入草稿吗？"));
        if (answer != QMessageBox::Yes) return;
    }
    m_inputs.clear();
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
    if (m_inputs.isEmpty()) {
        setStatus(QStringLiteral("请先添加至少 3 组手动位姿。"), true);
        return;
    }
    if (!canEditWithCurrentSpec()) return;
    emit applyRequested(m_inputs, m_draftSpec, false);
}

void ManualPosePage::applyAndCalculate()
{
    if (m_inputs.isEmpty()) {
        setStatus(QStringLiteral("请先添加至少 3 组手动位姿。"), true);
        return;
    }
    if (!canEditWithCurrentSpec()) return;
    emit applyRequested(m_inputs, m_draftSpec, true);
}

void ManualPosePage::onCurrentRowChanged(int row, int, int, int)
{
    showInput(row);
}

bool ManualPosePage::canEditWithCurrentSpec()
{
    if (m_hasPendingSpec) {
        setStatus(QStringLiteral("当前草稿使用旧的输入规范，请先清空草稿，再录入新规范的数据。"), true);
        return false;
    }
    return true;
}

void ManualPosePage::setStatus(const QString &text, bool warning)
{
    m_status->setText(text);
    m_status->setStyleSheet(warning ? QStringLiteral("color: #C0392B;") : QString{});
}

void ManualPosePage::updateSpecWidgets()
{
    const PoseInputSpec &activeSpec = m_hasDraftSpec ? m_draftSpec : m_inputSpec;
    m_specSummary->setText(QStringLiteral("当前输入规则：%1 | 内部统一为 Rodrigues(rad)+m")
                               .arg(poseSummary(activeSpec)));
    const QStringList labels = rotationLabels(activeSpec.rotationFormat);
    const bool quaternion = activeSpec.rotationFormat == RotationFormat::QuaternionWXYZ;
    for (int index = 0; index < m_tcpRotationLabels.size(); ++index) {
        const bool visible = index < (quaternion ? 4 : 3);
        m_tcpRotationLabels.at(index)->setText(index < labels.size() ? labels.at(index) : QStringLiteral("第4分量"));
        m_tcpRotationLabels.at(index)->setVisible(visible);
        m_tcpRotationEdits.at(index)->setVisible(visible);
    }
    for (int index = 0; index < m_cameraRotationLabels.size(); ++index) {
        const bool visible = index < (quaternion ? 4 : 3);
        m_cameraRotationLabels.at(index)->setText(index < labels.size() ? labels.at(index) : QStringLiteral("第4分量"));
        m_cameraRotationLabels.at(index)->setVisible(visible);
        m_cameraRotationEdits.at(index)->setVisible(visible);
    }
    if (!m_hasDraftSpec) {
        m_tcpRotationEdits.at(3)->clear();
        m_cameraRotationEdits.at(3)->clear();
    }
}

void ManualPosePage::updateTable()
{
    static const QStringList headers = {
        QStringLiteral("ID"), QStringLiteral("TCP Tx"), QStringLiteral("TCP Ty"), QStringLiteral("TCP Tz"),
        QStringLiteral("TCP R1"), QStringLiteral("TCP R2"), QStringLiteral("TCP R3"), QStringLiteral("TCP R4"),
        QStringLiteral("相机 Tx"), QStringLiteral("相机 Ty"), QStringLiteral("相机 Tz"),
        QStringLiteral("相机 R1"), QStringLiteral("相机 R2"), QStringLiteral("相机 R3"), QStringLiteral("相机 R4"),
        QStringLiteral("状态")};
    m_table->setUpdatesEnabled(false);
    m_table->clearContents();
    m_table->setHorizontalHeaderLabels(headers);
    m_table->setRowCount(m_inputs.size());
    const bool quaternion = (m_hasDraftSpec ? m_draftSpec : m_inputSpec).rotationFormat
                            == RotationFormat::QuaternionWXYZ;
    for (int row = 0; row < m_inputs.size(); ++row) {
        const ManualPoseInput &input = m_inputs.at(row);
        m_table->setItem(row, 0, new QTableWidgetItem(QString::number(input.id)));
        for (int index = 0; index < 3; ++index) {
            m_table->setItem(row, 1 + index, new QTableWidgetItem(formatValue(input.tcpTranslation[index])));
            m_table->setItem(row, 8 + index, new QTableWidgetItem(formatValue(input.cameraTranslation[index])));
        }
        for (int index = 0; index < 4; ++index) {
            m_table->setItem(row, 4 + index,
                             new QTableWidgetItem(quaternion || index < 3 ? formatValue(input.tcpRotation[index]) : QString{}));
            m_table->setItem(row, 11 + index,
                             new QTableWidgetItem(quaternion || index < 3 ? formatValue(input.cameraRotation[index]) : QString{}));
        }
        m_table->setItem(row, 15, new QTableWidgetItem(QStringLiteral("待应用")));
    }
    m_table->setUpdatesEnabled(true);
}

void ManualPosePage::showInput(int row)
{
    if (row < 0 || row >= m_inputs.size()) return;
    const ManualPoseInput &input = m_inputs.at(row);
    m_idEdit->setText(QString::number(input.id));
    for (int index = 0; index < 3; ++index) {
        m_tcpTranslationEdits.at(index)->setText(formatValue(input.tcpTranslation[index]));
        m_cameraTranslationEdits.at(index)->setText(formatValue(input.cameraTranslation[index]));
    }
    for (int index = 0; index < 4; ++index) {
        m_tcpRotationEdits.at(index)->setText(formatValue(input.tcpRotation[index]));
        m_cameraRotationEdits.at(index)->setText(formatValue(input.cameraRotation[index]));
    }
}

} // namespace handeye
