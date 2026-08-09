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
    return QStringLiteral("TCP：gripper→base | 相机：固定点 XYZ（camera 坐标系）| 旋转：%1 | 角度：%2 | 长度：%3")
        .arg(rotationFormatName(spec.rotationFormat), angleUnitName(spec.angleUnit),
             lengthUnitName(spec.lengthUnit));
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

QString formatValue(double value)
{
    return QString::number(value, 'g', 12);
}

} // namespace

ManualPosePage::ManualPosePage(QWidget *parent) : QWidget(parent)
{
    m_inputSpec = PoseInputSpec{};
    m_draftSpec = m_inputSpec;

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(20, 20, 20, 20);
    layout->setSpacing(12);

    auto *title = new QLabel(QStringLiteral("手动输入 TCP 与相机固定点"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *description = new QLabel(
        QStringLiteral("FixedPoint3D：每一组数据包含机器人 TCP 末端 6D 位姿和相机测得的同一个固定物理点 XYZ。\n"
                       "相机端不需要输入 rx/ry/rz；程序计算 camera→gripper，并评价每组预测的 base 固定点残差。"),
        this);
    description->setWordWrap(true);
    layout->addWidget(description);

    m_specSummary = new QLabel(this);
    m_specSummary->setObjectName(QStringLiteral("manualPoseSpecSummary"));
    m_specSummary->setWordWrap(true);
    layout->addWidget(m_specSummary);

    auto *inputGroup = new QGroupBox(QStringLiteral("新增或编辑一组点基样本"), this);
    auto *inputLayout = new QHBoxLayout(inputGroup);

    auto *tcpGroup = new QGroupBox(QStringLiteral("TCP 末端（gripper→base）"), inputGroup);
    auto *tcpForm = new QFormLayout(tcpGroup);
    m_idEdit = makeEdit(tcpGroup, QStringLiteral("例如 1"));
    tcpForm->addRow(QStringLiteral("样本 ID"), m_idEdit);
    for (const QString &name : {QStringLiteral("Tx"), QStringLiteral("Ty"), QStringLiteral("Tz")}) {
        auto *edit = makeEdit(tcpGroup, QStringLiteral("数值"));
        m_tcpTranslationEdits.append(edit);
        tcpForm->addRow(name, edit);
    }
    const QStringList labels = rotationLabels(m_inputSpec.rotationFormat);
    for (int index = 0; index < 4; ++index) {
        auto *label = new QLabel(index < labels.size() ? labels.at(index) : QStringLiteral("r4"), tcpGroup);
        auto *edit = makeEdit(tcpGroup, QStringLiteral("数值"));
        m_tcpRotationLabels.append(label);
        m_tcpRotationEdits.append(edit);
        tcpForm->addRow(label, edit);
    }
    inputLayout->addWidget(tcpGroup, 1);

    auto *pointGroup = new QGroupBox(QStringLiteral("相机固定点（camera 坐标系，仅 XYZ）"), inputGroup);
    auto *pointForm = new QFormLayout(pointGroup);
    for (const QString &name : {QStringLiteral("Xc"), QStringLiteral("Yc"), QStringLiteral("Zc")}) {
        auto *edit = makeEdit(pointGroup, QStringLiteral("数值"));
        m_cameraPointEdits.append(edit);
        pointForm->addRow(name, edit);
    }
    auto *pointHint = new QLabel(QStringLiteral("每组相机 XYZ 必须对应同一个固定物理点。"), pointGroup);
    pointHint->setWordWrap(true);
    pointForm->addRow(pointHint);
    inputLayout->addWidget(pointGroup, 1);

    auto *buttonLayout = new QVBoxLayout;
    auto *addButton = new QPushButton(QStringLiteral("添加一组"), inputGroup);
    addButton->setObjectName(QStringLiteral("addManualPointButton"));
    auto *updateButton = new QPushButton(QStringLiteral("更新选中"), inputGroup);
    updateButton->setObjectName(QStringLiteral("updateManualPointButton"));
    auto *clearButton = new QPushButton(QStringLiteral("清空输入"), inputGroup);
    clearButton->setObjectName(QStringLiteral("clearManualPointButton"));
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(updateButton);
    buttonLayout->addWidget(clearButton);
    buttonLayout->addStretch();
    inputLayout->addLayout(buttonLayout);
    layout->addWidget(inputGroup);

    m_table = new QTableWidget(this);
    m_table->setObjectName(QStringLiteral("manualPoseTable"));
    m_table->setColumnCount(11);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    layout->addWidget(m_table, 1);

    auto *actionLayout = new QHBoxLayout;
    auto *deleteButton = new QPushButton(QStringLiteral("删除选中"), this);
    deleteButton->setObjectName(QStringLiteral("deleteManualPointButton"));
    auto *applyButton = new QPushButton(QStringLiteral("应用到当前标定数据"), this);
    applyButton->setObjectName(QStringLiteral("applyManualPointButton"));
    applyButton->setProperty("variant", "primary");
    auto *calculateButton = new QPushButton(QStringLiteral("应用并计算五种算法"), this);
    calculateButton->setObjectName(QStringLiteral("applyAndCalculateManualPoseButton"));
    calculateButton->setToolTip(QStringLiteral("FixedPoint3D 模式下将执行点基标定；PosePairs 模式才执行五种 OpenCV 算法。"));
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

    m_status = new QLabel(QStringLiteral("尚未添加点基样本。"), this);
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

void ManualPosePage::setInputSpec(const PoseInputSpec &spec)
{
    if (!m_inputs.isEmpty() && !sameSpec(spec, m_draftSpec)) {
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

bool ManualPosePage::readInput(PointSample *input) const
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
    const bool quaternion = m_inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ;
    if (quaternion) return false;
    if (!readVector(m_tcpTranslationEdits, &input->gripperTranslation, 3)
        || !readVector(m_tcpRotationEdits, &input->gripperRotation, 3)
        || !readVector(m_cameraPointEdits, &input->cameraPoint, 3))
        return false;
    input->label = QStringLiteral("手动点基输入 #%1").arg(input->id);
    return true;
}

void ManualPosePage::addInput()
{
    if (!canEditWithCurrentSpec()) return;
    PointSample input;
    if (!readInput(&input)) {
        setStatus(QStringLiteral("请填写有效数字；FixedPoint3D 的 TCP 旋转格式需为三轴表示。"), true);
        return;
    }
    for (const PointSample &existing : m_inputs) {
        if (existing.id == input.id) {
            setStatus(QStringLiteral("样本 ID 已存在：%1，请使用更新或修改 ID。").arg(input.id), true);
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
    setStatus(QStringLiteral("已添加样本 %1，当前共 %2 组。").arg(input.id).arg(m_inputs.size()));
}

void ManualPosePage::updateInput()
{
    if (!canEditWithCurrentSpec()) return;
    const int row = m_table->currentRow();
    if (row < 0 || row >= m_inputs.size()) {
        setStatus(QStringLiteral("请先选择要更新的样本。"), true);
        return;
    }
    PointSample input;
    if (!readInput(&input)) {
        setStatus(QStringLiteral("请填写有效数字；FixedPoint3D 的 TCP 旋转格式需为三轴表示。"), true);
        return;
    }
    for (int index = 0; index < m_inputs.size(); ++index) {
        if (index != row && m_inputs.at(index).id == input.id) {
            setStatus(QStringLiteral("样本 ID 已存在：%1。").arg(input.id), true);
            return;
        }
    }
    m_inputs[row] = input;
    updateTable();
    m_table->selectRow(row);
    setStatus(QStringLiteral("已更新样本 %1。").arg(input.id));
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
    if (!m_inputs.isEmpty()) m_table->selectRow(std::min(row, static_cast<int>(m_inputs.size()) - 1));
    setStatus(QStringLiteral("已删除选中样本，当前剩余 %1 组。").arg(m_inputs.size()));
}

void ManualPosePage::clearInputs()
{
    if (!m_inputs.isEmpty()) {
        const auto answer = QMessageBox::question(this, QStringLiteral("清空手动数据"),
                                                  QStringLiteral("确定清空当前 FixedPoint3D 输入草稿吗？"));
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
    setStatus(QStringLiteral("已清空手动点基输入草稿。"));
}

void ManualPosePage::applyInputs()
{
    if (m_inputs.size() < 5) {
        setStatus(QStringLiteral("点基标定至少需要 5 组样本。"), true);
        return;
    }
    if (!canEditWithCurrentSpec()) return;
    emit applyPointRequested(m_inputs, m_draftSpec, false);
}

void ManualPosePage::applyAndCalculate()
{
    if (m_inputs.size() < 5) {
        setStatus(QStringLiteral("点基标定至少需要 5 组样本。"), true);
        return;
    }
    if (!canEditWithCurrentSpec()) return;
    emit applyPointRequested(m_inputs, m_draftSpec, true);
}

void ManualPosePage::onCurrentRowChanged(int row, int, int, int)
{
    showInput(row);
}

bool ManualPosePage::canEditWithCurrentSpec()
{
    if (m_hasPendingSpec) {
        setStatus(QStringLiteral("当前草稿使用旧输入规范，请先清空草稿，再使用新规范录入。"), true);
        return false;
    }
    if (m_inputSpec.rotationFormat == RotationFormat::QuaternionWXYZ) {
        setStatus(QStringLiteral("FixedPoint3D 页面当前使用 TCP 三轴旋转输入，请在参数页选择 Rodrigues、Euler XYZ 或 RPY。"), true);
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
    m_specSummary->setText(QStringLiteral("当前输入规范：%1\n内部统一为 Rodrigues(rad)+m；相机端固定点 XYZ 使用长度单位 %2。")
                               .arg(poseSummary(activeSpec), lengthUnitName(activeSpec.lengthUnit)));
    const QStringList labels = rotationLabels(activeSpec.rotationFormat);
    const bool visible = activeSpec.rotationFormat != RotationFormat::QuaternionWXYZ;
    for (int index = 0; index < m_tcpRotationLabels.size(); ++index) {
        m_tcpRotationLabels.at(index)->setText(index < labels.size() ? labels.at(index) : QStringLiteral("r4"));
        m_tcpRotationLabels.at(index)->setVisible(index < 3 && visible);
        m_tcpRotationEdits.at(index)->setVisible(index < 3 && visible);
    }
}

void ManualPosePage::updateTable()
{
    static const QStringList headers = {
        QStringLiteral("ID"), QStringLiteral("TCP Tx"), QStringLiteral("TCP Ty"), QStringLiteral("TCP Tz"),
        QStringLiteral("TCP R1"), QStringLiteral("TCP R2"), QStringLiteral("TCP R3"),
        QStringLiteral("Camera X"), QStringLiteral("Camera Y"), QStringLiteral("Camera Z"), QStringLiteral("状态")};
    m_table->setUpdatesEnabled(false);
    m_table->clearContents();
    m_table->setHorizontalHeaderLabels(headers);
    m_table->setRowCount(m_inputs.size());
    for (int row = 0; row < m_inputs.size(); ++row) {
        const PointSample &input = m_inputs.at(row);
        m_table->setItem(row, 0, new QTableWidgetItem(QString::number(input.id)));
        for (int index = 0; index < 3; ++index) {
            m_table->setItem(row, 1 + index, new QTableWidgetItem(formatValue(input.gripperTranslation[index])));
            m_table->setItem(row, 4 + index, new QTableWidgetItem(formatValue(input.gripperRotation[index])));
            m_table->setItem(row, 7 + index, new QTableWidgetItem(formatValue(input.cameraPoint[index])));
        }
        m_table->setItem(row, 10, new QTableWidgetItem(QStringLiteral("待应用")));
    }
    m_table->setUpdatesEnabled(true);
}

void ManualPosePage::showInput(int row)
{
    if (row < 0 || row >= m_inputs.size()) return;
    const PointSample &input = m_inputs.at(row);
    m_idEdit->setText(QString::number(input.id));
    for (int index = 0; index < 3; ++index) {
        m_tcpTranslationEdits.at(index)->setText(formatValue(input.gripperTranslation[index]));
        m_tcpRotationEdits.at(index)->setText(formatValue(input.gripperRotation[index]));
        m_cameraPointEdits.at(index)->setText(formatValue(input.cameraPoint[index]));
    }
}

} // namespace handeye
