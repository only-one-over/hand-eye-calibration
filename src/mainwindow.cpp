#include "mainwindow.h"

#include "core/calibration_service.h"
#include "core/dataset_validator.h"
#include "core/synthetic_dataset.h"
#include "io/dataset_io.h"

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
#include <QStatusBar>
#include <QTableView>
#include <QVBoxLayout>

#include <utility>

namespace handeye {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("手眼标定工具 - Qt6"));
    resize(1320, 820);
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
    controls->setMinimumWidth(230);
    auto *controlsLayout = new QVBoxLayout(controls);

    auto *dataGroup = new QGroupBox(QStringLiteral("数据与参数"), controls);
    auto *form = new QFormLayout(dataGroup);
    m_modeCombo = new QComboBox(dataGroup);
    m_modeCombo->addItem(QStringLiteral("眼在手 (camera → gripper)"));
    m_modeCombo->addItem(QStringLiteral("眼在外 (输入方向适配预留)"));
    m_methodCombo = new QComboBox(dataGroup);
    for (CalibrationMethod method : allMethods())
        m_methodCombo->addItem(methodName(method));
    m_unitEdit = new QLineEdit(QStringLiteral("m"), dataGroup);
    form->addRow(QStringLiteral("模式"), m_modeCombo);
    form->addRow(QStringLiteral("算法"), m_methodCombo);
    form->addRow(QStringLiteral("平移单位"), m_unitEdit);
    controlsLayout->addWidget(dataGroup);

    auto *actions = new QGroupBox(QStringLiteral("操作"), controls);
    auto *actionsLayout = new QVBoxLayout(actions);
    auto addButton = [actionsLayout, actions](const QString &text) {
        auto *button = new QPushButton(text, actions);
        actionsLayout->addWidget(button);
        return button;
    };
    connect(addButton(QStringLiteral("生成示例数据")), &QPushButton::clicked, this, &MainWindow::generateDemo);
    connect(addButton(QStringLiteral("导入 CSV")), &QPushButton::clicked, this, &MainWindow::importCsv);
    connect(addButton(QStringLiteral("导入 JSON")), &QPushButton::clicked, this, &MainWindow::importJson);
    connect(addButton(QStringLiteral("计算当前算法")), &QPushButton::clicked, this, &MainWindow::calculateSelected);
    connect(addButton(QStringLiteral("五种算法全部计算")), &QPushButton::clicked, this, &MainWindow::calculateAll);
    connect(addButton(QStringLiteral("删除选中样本")), &QPushButton::clicked, this, &MainWindow::deleteSelectedSamples);
    controlsLayout->addWidget(actions);
    controlsLayout->addStretch();

    auto *right = new QWidget(splitter);
    auto *rightLayout = new QVBoxLayout(right);
    rightLayout->addWidget(new QLabel(QStringLiteral("样本数据（旋转为 Rodrigues 弧度，平移使用上方单位）"), right));
    m_sampleTable = new QTableView(right);
    m_sampleTable->setModel(m_sampleModel);
    m_sampleTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_sampleTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_sampleTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    rightLayout->addWidget(m_sampleTable, 3);
    rightLayout->addWidget(new QLabel(QStringLiteral("算法结果"), right));
    m_resultTable = new QTableView(right);
    m_resultTable->setModel(m_resultModel);
    m_resultTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_resultTable->horizontalHeader()->setStretchLastSection(true);
    rightLayout->addWidget(m_resultTable, 2);

    auto *bottom = new QSplitter(Qt::Horizontal, right);
    m_matrixView = new QPlainTextEdit(bottom);
    m_matrixView->setReadOnly(true);
    m_matrixView->setPlaceholderText(QStringLiteral("选择算法结果后显示 4×4 变换矩阵"));
    m_logView = new QPlainTextEdit(bottom);
    m_logView->setReadOnly(true);
    m_logView->setPlaceholderText(QStringLiteral("运行日志"));
    bottom->setMinimumHeight(150);
    rightLayout->addWidget(bottom, 1);

    splitter->addWidget(controls);
    splitter->addWidget(right);
    splitter->setStretchFactor(1, 1);
    rootLayout->addWidget(splitter);
    setCentralWidget(root);

    auto *fileMenu = menuBar()->addMenu(QStringLiteral("文件"));
    auto *newAction = fileMenu->addAction(QStringLiteral("新建"));
    auto *csvInAction = fileMenu->addAction(QStringLiteral("导入 CSV"));
    auto *jsonInAction = fileMenu->addAction(QStringLiteral("导入 JSON"));
    fileMenu->addSeparator();
    auto *csvOutAction = fileMenu->addAction(QStringLiteral("导出 CSV"));
    auto *jsonOutAction = fileMenu->addAction(QStringLiteral("导出 JSON"));
    connect(newAction, &QAction::triggered, this, &MainWindow::newDataset);
    connect(csvInAction, &QAction::triggered, this, &MainWindow::importCsv);
    connect(jsonInAction, &QAction::triggered, this, &MainWindow::importJson);
    connect(csvOutAction, &QAction::triggered, this, &MainWindow::exportCsv);
    connect(jsonOutAction, &QAction::triggered, this, &MainWindow::exportJson);
    connect(m_resultTable, &QTableView::clicked, this, &MainWindow::showSelectedResult);
}

void MainWindow::syncSamplesToView()
{
    m_sampleModel->setSamples(m_dataset.samples);
    m_resultModel->setResults(m_dataset.results);
    statusBar()->showMessage(QStringLiteral("当前样本数：%1").arg(m_dataset.samples.size()));
}

void MainWindow::appendLog(const QString &message)
{
    m_logView->appendPlainText(QStringLiteral("[%1] %2")
                                   .arg(QDateTime::currentDateTime().toString("HH:mm:ss"), message));
}

CalibrationMethod MainWindow::selectedMethod() const
{
    const QVector<CalibrationMethod> methods = allMethods();
    return methods.value(m_methodCombo->currentIndex(), CalibrationMethod::Tsai);
}

void MainWindow::newDataset()
{
    m_dataset = CalibrationDataset{};
    m_dataset.translationUnit = m_unitEdit->text().trimmed();
    syncSamplesToView();
    appendLog(QStringLiteral("已新建空数据集。"));
}

void MainWindow::generateDemo()
{
    m_dataset = makeSyntheticDataset();
    syncSamplesToView();
    appendLog(QStringLiteral("已生成 %1 组确定性示例数据。建议点击“五种算法全部计算”。")
                  .arg(m_dataset.samples.size()));
}

void MainWindow::importCsv()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入 CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    const IoResult result = readCsv(path, &m_dataset);
    if (!result.success) { QMessageBox::warning(this, QStringLiteral("导入失败"), result.error); return; }
    syncSamplesToView(); appendLog(QStringLiteral("已导入 CSV：%1").arg(path));
}

void MainWindow::importJson()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入 JSON"), {}, QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    const IoResult result = readJson(path, &m_dataset);
    if (!result.success) { QMessageBox::warning(this, QStringLiteral("导入失败"), result.error); return; }
    syncSamplesToView(); appendLog(QStringLiteral("已导入 JSON：%1").arg(path));
}

void MainWindow::exportCsv()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出 CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    const IoResult result = writeCsv(path, m_dataset);
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出 CSV：%1").arg(path));
}

void MainWindow::exportJson()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出 JSON"), {}, QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    const IoResult result = writeJson(path, m_dataset);
    if (!result.success) QMessageBox::warning(this, QStringLiteral("导出失败"), result.error);
    else appendLog(QStringLiteral("已导出 JSON：%1").arg(path));
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
    appendLog(QStringLiteral("已删除 %1 组样本，结果已清空。").arg(ids.size()));
}

void MainWindow::calculateSelected()
{
    m_dataset.translationUnit = m_unitEdit->text().trimmed();
    m_dataset.mode = m_modeCombo->currentIndex() == 0 ? CalibrationMode::EyeInHand : CalibrationMode::EyeToHand;
    const ValidationReport validation = validateDataset(m_dataset);
    if (!validation.valid) { QMessageBox::warning(this, QStringLiteral("无法计算"), validation.errors.join('\n')); return; }
    const CalibrationResult result = CalibrationService::calibrate(m_dataset, selectedMethod());
    m_dataset.results = {result};
    syncSamplesToView();
    appendLog(QStringLiteral("%1：%2（耗时 %3 ms）").arg(methodName(result.method), result.message).arg(result.elapsedMs));
    updateMatrixView(result);
}

void MainWindow::calculateAll()
{
    m_dataset.translationUnit = m_unitEdit->text().trimmed();
    m_dataset.mode = m_modeCombo->currentIndex() == 0 ? CalibrationMode::EyeInHand : CalibrationMode::EyeToHand;
    const ValidationReport validation = validateDataset(m_dataset);
    if (!validation.valid) { QMessageBox::warning(this, QStringLiteral("无法计算"), validation.errors.join('\n')); return; }
    m_dataset.results = CalibrationService::calibrateAll(m_dataset);
    syncSamplesToView();
    for (const CalibrationResult &result : std::as_const(m_dataset.results))
        appendLog(QStringLiteral("%1：%2，旋转误差 %3°，平移误差 %4")
                      .arg(methodName(result.method), result.message)
                      .arg(result.rotationErrorDeg, 0, 'f', 4)
                      .arg(result.translationError, 0, 'f', 6));
    if (!m_dataset.results.isEmpty()) updateMatrixView(m_dataset.results.first());
}

void MainWindow::showSelectedResult(const QModelIndex &index)
{
    updateMatrixView(m_resultModel->resultAt(index.row()));
}

void MainWindow::updateMatrixView(const CalibrationResult &result)
{
    if (!result.success) { m_matrixView->setPlainText(result.message); return; }
    QStringList lines;
    lines << QStringLiteral("%1  |  camera → gripper").arg(methodName(result.method));
    for (const auto &row : result.cameraToGripper)
        lines << QStringLiteral("[%1, %2, %3, %4]")
                      .arg(row[0], 0, 'f', 8).arg(row[1], 0, 'f', 8)
                      .arg(row[2], 0, 'f', 8).arg(row[3], 0, 'f', 8);
    m_matrixView->setPlainText(lines.join('\n'));
}

} // namespace handeye
