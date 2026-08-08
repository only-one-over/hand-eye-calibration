#include "mainwindow.h"

#include "core/calibration_service.h"
#include "core/dataset_validator.h"
#include "core/pose_conversion.h"
#include "core/synthetic_dataset.h"
#include "io/dataset_io.h"
#include "io/pose_adapter.h"

#include <QAction>
#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QStandardItemModel>
#include <QStatusBar>
#include <QTableView>
#include <QVBoxLayout>

#include <utility>

namespace handeye {

namespace {

RotationFormat formatFromIndex(int index)
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

} // namespace

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("手眼标定工具 - Qt6"));
    resize(1480, 900);
    setupUi();
    generateDemo();
}

void MainWindow::setupUi()
{
    m_sampleModel = new SampleTableModel(this);
    m_resultModel = new ResultTableModel(this);

    auto *root = new QWidget(this);
    auto *rootLayout = new QVBoxLayout(root);
    auto *splitter = new QSplitter(Qt::Horizontal, root);
    auto *controls = new QWidget(splitter);
    controls->setMinimumWidth(280);
    auto *controlsLayout = new QVBoxLayout(controls);

    auto *dataGroup = new QGroupBox(QStringLiteral("数据与标准化"), controls);
    auto *form = new QFormLayout(dataGroup);
    m_modeCombo = new QComboBox(dataGroup);
    m_modeCombo->addItem(QStringLiteral("Eye-In-Hand（眼在手）"));
    m_modeCombo->addItem(QStringLiteral("Eye-To-Hand（暂未启用）"));
    if (auto *model = qobject_cast<QStandardItemModel *>(m_modeCombo->model()))
        model->item(1)->setEnabled(false);
    m_adapterCombo = new QComboBox(dataGroup);
    for (int index = 0; index < 4; ++index)
        m_adapterCombo->addItem(adapterName(adapterFromIndex(index)));
    m_rotationFormatCombo = new QComboBox(dataGroup);
    m_rotationFormatCombo->addItems({QStringLiteral("Rodrigues"), QStringLiteral("Euler XYZ"),
                                     QStringLiteral("RPY (Z-Y-X)"), QStringLiteral("Quaternion (w,x,y,z)")});
    m_angleUnitCombo = new QComboBox(dataGroup);
    m_angleUnitCombo->addItems({QStringLiteral("弧度 rad"), QStringLiteral("角度 degree")});
    m_lengthUnitCombo = new QComboBox(dataGroup);
    m_lengthUnitCombo->addItems({QStringLiteral("米 m"), QStringLiteral("毫米 mm")});
    m_robotEdit = new QLineEdit(QStringLiteral("未指定机器人"), dataGroup);
    m_cameraEdit = new QLineEdit(QStringLiteral("未指定相机"), dataGroup);
    m_unitEdit = new QLineEdit(QStringLiteral("内部标准：Rodrigues(rad) + m"), dataGroup);
    m_unitEdit->setReadOnly(true);
    m_methodCombo = new QComboBox(dataGroup);
    for (CalibrationMethod method : allMethods()) m_methodCombo->addItem(methodName(method));
    form->addRow(QStringLiteral("标定模式"), m_modeCombo);
    form->addRow(QStringLiteral("Pose Adapter"), m_adapterCombo);
    form->addRow(QStringLiteral("输入旋转"), m_rotationFormatCombo);
    form->addRow(QStringLiteral("角度单位"), m_angleUnitCombo);
    form->addRow(QStringLiteral("平移单位"), m_lengthUnitCombo);
    form->addRow(QStringLiteral("机器人"), m_robotEdit);
    form->addRow(QStringLiteral("相机"), m_cameraEdit);
    form->addRow(QStringLiteral("算法"), m_methodCombo);
    form->addRow(QStringLiteral("内部格式"), m_unitEdit);
    controlsLayout->addWidget(dataGroup);

    auto *actions = new QGroupBox(QStringLiteral("数据与计算"), controls);
    auto *actionsLayout = new QVBoxLayout(actions);
    auto addButton = [actionsLayout, actions](const QString &text) {
        auto *button = new QPushButton(text, actions);
        actionsLayout->addWidget(button);
        return button;
    };
    connect(addButton(QStringLiteral("生成合成真值数据")), &QPushButton::clicked, this, &MainWindow::generateDemo);
    connect(addButton(QStringLiteral("导入训练 CSV")), &QPushButton::clicked, this, &MainWindow::importCsv);
    connect(addButton(QStringLiteral("导入独立验证 CSV")), &QPushButton::clicked, this, &MainWindow::importValidationCsv);
    connect(addButton(QStringLiteral("导入 JSON")), &QPushButton::clicked, this, &MainWindow::importJson);
    connect(addButton(QStringLiteral("计算当前算法")), &QPushButton::clicked, this, &MainWindow::calculateSelected);
    connect(addButton(QStringLiteral("五种算法自动比较并推荐")), &QPushButton::clicked, this, &MainWindow::calculateAll);
    connect(addButton(QStringLiteral("删除选中样本")), &QPushButton::clicked, this, &MainWindow::deleteSelectedSamples);
    controlsLayout->addWidget(actions);

    m_reliabilityLabel = new QLabel(controls);
    m_reliabilityLabel->setWordWrap(true);
    m_reliabilityLabel->setText(QStringLiteral("可靠性报告将在计算后显示。\n通过阈值：旋转 RMSE ≤ 0.5°，平移 RMSE ≤ 1 mm"));
    controlsLayout->addWidget(m_reliabilityLabel);
    controlsLayout->addStretch();

    auto *right = new QWidget(splitter);
    auto *rightLayout = new QVBoxLayout(right);
    rightLayout->addWidget(new QLabel(QStringLiteral(
        "样本数据：输入格式只用于边界转换；内部统一为 Rodrigues(rad)、米(m)。方向：gripper→base + target→camera，输出 camera→gripper。"), right));
    m_sampleTable = new QTableView(right);
    m_sampleTable->setModel(m_sampleModel);
    m_sampleTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_sampleTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_sampleTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_sampleTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    rightLayout->addWidget(m_sampleTable, 4);
    rightLayout->addWidget(new QLabel(QStringLiteral("算法结果（RMSE 为真实平方均方根误差）"), right));
    m_resultTable = new QTableView(right);
    m_resultTable->setModel(m_resultModel);
    m_resultTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_resultTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_resultTable->horizontalHeader()->setStretchLastSection(true);
    rightLayout->addWidget(m_resultTable, 2);

    auto *bottom = new QSplitter(Qt::Horizontal, right);
    m_matrixView = new QPlainTextEdit(bottom);
    m_matrixView->setReadOnly(true);
    m_matrixView->setPlaceholderText(QStringLiteral("选择算法结果后显示 4×4 变换矩阵"));
    m_logView = new QPlainTextEdit(bottom);
    m_logView->setReadOnly(true);
    m_logView->setPlaceholderText(QStringLiteral("运行日志"));
    bottom->setMinimumHeight(170);
    rightLayout->addWidget(bottom, 1);

    splitter->addWidget(controls);
    splitter->addWidget(right);
    splitter->setStretchFactor(1, 1);
    rootLayout->addWidget(splitter);
    setCentralWidget(root);

    auto *fileMenu = menuBar()->addMenu(QStringLiteral("文件"));
    auto *newAction = fileMenu->addAction(QStringLiteral("新建"));
    auto *csvInAction = fileMenu->addAction(QStringLiteral("导入训练 CSV"));
    auto *validationInAction = fileMenu->addAction(QStringLiteral("导入独立验证 CSV"));
    auto *jsonInAction = fileMenu->addAction(QStringLiteral("导入 JSON"));
    fileMenu->addSeparator();
    auto *csvOutAction = fileMenu->addAction(QStringLiteral("导出训练 CSV"));
    auto *jsonOutAction = fileMenu->addAction(QStringLiteral("导出 JSON"));
    auto *yamlOutAction = fileMenu->addAction(QStringLiteral("导出 YAML"));
    auto *txtOutAction = fileMenu->addAction(QStringLiteral("导出 TXT 矩阵"));
    auto *cppOutAction = fileMenu->addAction(QStringLiteral("导出 C++ 矩阵"));
    auto *pythonOutAction = fileMenu->addAction(QStringLiteral("导出 Python 矩阵"));
    connect(newAction, &QAction::triggered, this, &MainWindow::newDataset);
    connect(csvInAction, &QAction::triggered, this, &MainWindow::importCsv);
    connect(validationInAction, &QAction::triggered, this, &MainWindow::importValidationCsv);
    connect(jsonInAction, &QAction::triggered, this, &MainWindow::importJson);
    connect(csvOutAction, &QAction::triggered, this, &MainWindow::exportCsv);
    connect(jsonOutAction, &QAction::triggered, this, &MainWindow::exportJson);
    connect(yamlOutAction, &QAction::triggered, this, &MainWindow::exportYaml);
    connect(txtOutAction, &QAction::triggered, this, &MainWindow::exportTxt);
    connect(cppOutAction, &QAction::triggered, this, &MainWindow::exportCpp);
    connect(pythonOutAction, &QAction::triggered, this, &MainWindow::exportPython);
    connect(m_resultTable, &QTableView::clicked, this, &MainWindow::showSelectedResult);
    connect(m_adapterCombo, &QComboBox::currentIndexChanged, this, [this](int index) {
        const PoseInputSpec spec = pose::defaultSpec(adapterFromIndex(index));
        m_rotationFormatCombo->setCurrentIndex(static_cast<int>(spec.rotationFormat));
        m_angleUnitCombo->setCurrentIndex(spec.angleUnit == AngleUnit::Degrees ? 1 : 0);
        m_lengthUnitCombo->setCurrentIndex(spec.lengthUnit == LengthUnit::Millimeters ? 1 : 0);
    });
}

void MainWindow::updateInputSpecFromUi()
{
    m_dataset.inputSpec.adapter = adapterFromIndex(m_adapterCombo->currentIndex());
    m_dataset.inputSpec.rotationFormat = formatFromIndex(m_rotationFormatCombo->currentIndex());
    m_dataset.inputSpec.angleUnit = angleUnitFromIndex(m_angleUnitCombo->currentIndex());
    m_dataset.inputSpec.lengthUnit = lengthUnitFromIndex(m_lengthUnitCombo->currentIndex());
    m_dataset.inputSpec.direction = PoseDirection::GripperToBase;
    m_dataset.robotName = m_robotEdit->text().trimmed();
    m_dataset.cameraName = m_cameraEdit->text().trimmed();
}

void MainWindow::syncSamplesToView()
{
    m_sampleModel->setSamples(m_dataset.samples);
    m_resultModel->setResults(m_dataset.results);
    statusBar()->showMessage(QStringLiteral("训练样本：%1，独立验证样本：%2")
                                 .arg(m_dataset.samples.size()).arg(m_dataset.validationSamples.size()));
}

void MainWindow::appendLog(const QString &message)
{
    m_logView->appendPlainText(QStringLiteral("[%1] %2")
                                   .arg(QDateTime::currentDateTime().toString("HH:mm:ss"), message));
}

CalibrationMethod MainWindow::selectedMethod() const
{
    return allMethods().value(m_methodCombo->currentIndex(), CalibrationMethod::Tsai);
}

CalibrationResult MainWindow::selectedResult() const
{
    const QModelIndexList selected = m_resultTable->selectionModel()->selectedRows();
    if (!selected.isEmpty()) return m_resultModel->resultAt(selected.first().row());
    for (const CalibrationResult &result : m_dataset.results)
        if (result.recommended) return result;
    return m_dataset.results.isEmpty() ? CalibrationResult{} : m_dataset.results.first();
}

void MainWindow::newDataset()
{
    const PoseInputSpec spec = m_dataset.inputSpec;
    m_dataset = CalibrationDataset{};
    m_dataset.inputSpec = spec;
    updateInputSpecFromUi();
    syncSamplesToView();
    m_reliabilityLabel->setText(QStringLiteral("可靠性报告将在计算后显示。"));
    appendLog(QStringLiteral("已新建空数据集。"));
}

void MainWindow::generateDemo()
{
    m_dataset = makeSyntheticDataset();
    syncSamplesToView();
    appendLog(QStringLiteral("已生成 %1 组带真值的合成数据；方向和数值可由 smoke test 验证。")
                  .arg(m_dataset.samples.size()));
}

void MainWindow::importCsv()
{
    updateInputSpecFromUi();
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入训练 CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    const IoResult result = readCsv(path, &m_dataset, m_dataset.inputSpec);
    if (!result.success) { QMessageBox::warning(this, QStringLiteral("导入失败"), result.error); return; }
    syncSamplesToView(); appendLog(QStringLiteral("已导入训练 CSV：%1，已标准化为 rad/m").arg(path));
}

void MainWindow::importValidationCsv()
{
    updateInputSpecFromUi();
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入独立验证 CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    CalibrationDataset validationData;
    const IoResult result = readCsv(path, &validationData, m_dataset.inputSpec);
    if (!result.success) { QMessageBox::warning(this, QStringLiteral("导入失败"), result.error); return; }
    m_dataset.validationSamples = validationData.samples;
    m_dataset.results.clear();
    syncSamplesToView(); appendLog(QStringLiteral("已导入独立验证 CSV：%1").arg(path));
}

void MainWindow::importJson()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入 JSON"), {}, QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    const IoResult result = readJson(path, &m_dataset);
    if (!result.success) { QMessageBox::warning(this, QStringLiteral("导入失败"), result.error); return; }
    m_robotEdit->setText(m_dataset.robotName);
    m_cameraEdit->setText(m_dataset.cameraName);
    syncSamplesToView(); appendLog(QStringLiteral("已导入 JSON：%1").arg(path));
}

void MainWindow::exportCsv()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出训练 CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    const IoResult result = writeCsv(path, m_dataset);
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出训练 CSV：%1（canonical Rodrigues/rad/m）").arg(path));
}

void MainWindow::exportJson()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出 JSON"), {}, QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    const IoResult result = writeJson(path, m_dataset);
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出 JSON：%1").arg(path));
}

void MainWindow::exportYaml()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出 YAML"), {}, QStringLiteral("YAML (*.yaml *.yml)"));
    if (path.isEmpty()) return;
    const IoResult result = writeYaml(path, m_dataset);
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出 YAML：%1").arg(path));
}

void MainWindow::exportTxt()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出 TXT 矩阵"), {}, QStringLiteral("TXT (*.txt)"));
    if (path.isEmpty()) return;
    const IoResult result = writeResultTxt(path, m_dataset, selectedResult());
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出 TXT 矩阵：%1").arg(path));
}

void MainWindow::exportCpp()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出 C++ 矩阵"), {}, QStringLiteral("C++ (*.cpp *.h)"));
    if (path.isEmpty()) return;
    const IoResult result = writeResultCpp(path, m_dataset, selectedResult());
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出 C++ 矩阵：%1").arg(path));
}

void MainWindow::exportPython()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出 Python 矩阵"), {}, QStringLiteral("Python (*.py)"));
    if (path.isEmpty()) return;
    const IoResult result = writeResultPython(path, m_dataset, selectedResult());
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出 Python 矩阵：%1").arg(path));
}

void MainWindow::deleteSelectedSamples()
{
    const QVector<int> ids = m_sampleModel->idsAt(m_sampleTable->selectionModel()->selectedRows());
    if (ids.isEmpty()) return;
    QVector<PoseSample> kept;
    for (const PoseSample &sample : std::as_const(m_dataset.samples))
        if (!ids.contains(sample.id)) kept.append(sample);
    m_dataset.samples = kept;
    m_dataset.results.clear();
    syncSamplesToView();
    appendLog(QStringLiteral("已删除 %1 组样本，结果已清空。合成真值标记已失效。").arg(ids.size()));
    m_dataset.hasGroundTruth = false;
}

void MainWindow::applyResidualsToSamples(const ReliabilityReport &report)
{
    for (PoseSample &sample : m_dataset.samples) {
        for (const SampleResidual &residual : report.sampleResiduals) {
            if (residual.sampleId == sample.id) {
                sample.rotationResidualDeg = residual.rotationErrorDeg;
                sample.translationResidualM = residual.translationErrorM;
                sample.outlier = residual.outlier;
                break;
            }
        }
    }
}

void MainWindow::showReliability(const CalibrationResult &result)
{
    const ReliabilityReport &training = result.trainingReport;
    QString text = QStringLiteral("训练：RMSE 旋转 %1°，平移 %2 m\n平均：%3° / %4 m\n最大：%5° / %6 m\n异常样本：%7\n可靠性：%8")
                       .arg(training.rotationRmseDeg, 0, 'f', 5).arg(training.translationRmseM, 0, 'f', 7)
                       .arg(training.rotationMeanDeg, 0, 'f', 5).arg(training.translationMeanM, 0, 'f', 7)
                       .arg(training.rotationMaxDeg, 0, 'f', 5).arg(training.translationMaxM, 0, 'f', 7)
                       .arg(training.outlierCount)
                       .arg(training.passed ? QStringLiteral("通过") : QStringLiteral("未通过"));
    if (result.validationReport.available)
        text += QStringLiteral("\n验证：RMSE 旋转 %1°，平移 %2 m，%3")
                    .arg(result.validationReport.rotationRmseDeg, 0, 'f', 5)
                    .arg(result.validationReport.translationRmseM, 0, 'f', 7)
                    .arg(result.validationReport.passed ? QStringLiteral("通过") : QStringLiteral("未通过"));
    if (result.recommended) text += QStringLiteral("\n算法推荐：%1").arg(methodName(result.method));
    m_reliabilityLabel->setText(text);
}

void MainWindow::calculateSelected()
{
    updateInputSpecFromUi();
    m_dataset.mode = CalibrationMode::EyeInHand;
    const ValidationReport validation = validateDataset(m_dataset);
    if (!validation.valid) { QMessageBox::warning(this, QStringLiteral("无法计算"), validation.errors.join('\n')); return; }
    const CalibrationResult result = CalibrationService::calibrate(m_dataset, selectedMethod());
    m_dataset.results = {result};
    applyResidualsToSamples(result.trainingReport);
    syncSamplesToView();
    showReliability(result);
    appendLog(QStringLiteral("%1：%2（耗时 %3 ms）").arg(methodName(result.method), result.message).arg(result.elapsedMs));
    updateMatrixView(result);
}

void MainWindow::calculateAll()
{
    updateInputSpecFromUi();
    m_dataset.mode = CalibrationMode::EyeInHand;
    const ValidationReport validation = validateDataset(m_dataset);
    if (!validation.valid) { QMessageBox::warning(this, QStringLiteral("无法计算"), validation.errors.join('\n')); return; }
    m_dataset.results = CalibrationService::calibrateAll(m_dataset);
    if (!m_dataset.results.isEmpty()) {
        CalibrationResult recommended = m_dataset.results.first();
        for (const CalibrationResult &candidate : std::as_const(m_dataset.results))
            if (candidate.recommended) { recommended = candidate; break; }
        applyResidualsToSamples(recommended.trainingReport);
    }
    syncSamplesToView();
    int recommendedRow = -1;
    for (int index = 0; index < m_dataset.results.size(); ++index) {
        const CalibrationResult &result = m_dataset.results.at(index);
        appendLog(QStringLiteral("%1：%2，RMSE %3° / %4 m，%5")
                      .arg(methodName(result.method), result.message)
                      .arg(result.trainingReport.rotationRmseDeg, 0, 'f', 5)
                      .arg(result.trainingReport.translationRmseM, 0, 'f', 7)
                      .arg(result.trainingReport.passed ? QStringLiteral("通过") : QStringLiteral("未通过")));
        if (result.recommended) recommendedRow = index;
    }
    if (recommendedRow >= 0) m_resultTable->selectRow(recommendedRow);
    const CalibrationResult result = recommendedRow >= 0 ? m_dataset.results.at(recommendedRow) : selectedResult();
    showReliability(result);
    if (!m_dataset.results.isEmpty()) updateMatrixView(result);
}

void MainWindow::showSelectedResult(const QModelIndex &index)
{
    const CalibrationResult result = m_resultModel->resultAt(index.row());
    showReliability(result);
    updateMatrixView(result);
}

void MainWindow::updateMatrixView(const CalibrationResult &result)
{
    if (!result.success) { m_matrixView->setPlainText(result.message); return; }
    QStringList lines;
    lines << QStringLiteral("%1  |  camera → gripper").arg(methodName(result.method));
    for (const auto &row : result.cameraToGripper)
        lines << QStringLiteral("[%1, %2, %3, %4]")
                      .arg(row[0], 0, 'f', 10).arg(row[1], 0, 'f', 10)
                      .arg(row[2], 0, 'f', 10).arg(row[3], 0, 'f', 10);
    m_matrixView->setPlainText(lines.join('\n'));
}

} // namespace handeye
