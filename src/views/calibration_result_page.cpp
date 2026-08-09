#include "views/calibration_result_page.h"

#include "models/calibration_session_model.h"

#include <QHBoxLayout>
#include <QAbstractItemView>
#include <QFont>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>

#include <QPair>
#include <QVector>

namespace handeye {

CalibrationResultPage::CalibrationResultPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(18, 18, 18, 18);
    auto *title = new QLabel(QStringLiteral("标定结果"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *actions = new QHBoxLayout;
    auto *calculateSelected = new QPushButton(QStringLiteral("计算当前算法"), this);
    calculateSelected->setObjectName(QStringLiteral("calculateSelectedButton"));
    auto *calculateAll = new QPushButton(QStringLiteral("五种算法自动比较并推荐"), this);
    calculateAll->setObjectName(QStringLiteral("calculateAllButton"));
    calculateAll->setProperty("variant", "primary");
    auto *importValidation = new QPushButton(QStringLiteral("导入独立验证数据"), this);
    actions->addWidget(calculateSelected);
    actions->addWidget(calculateAll);
    actions->addWidget(importValidation);
    actions->addStretch();
    layout->addLayout(actions);
    connect(calculateSelected, &QPushButton::clicked, this, &CalibrationResultPage::calculateSelectedRequested);
    connect(calculateAll, &QPushButton::clicked, this, &CalibrationResultPage::calculateAllRequested);
    connect(importValidation, &QPushButton::clicked, this, &CalibrationResultPage::importValidationRequested);

    auto *exports = new QHBoxLayout;
    const QVector<QPair<QString, QString>> exportButtons = {
        {QStringLiteral("JSON"), QStringLiteral("json")}, {QStringLiteral("YAML"), QStringLiteral("yaml")},
        {QStringLiteral("TXT"), QStringLiteral("txt")}, {QStringLiteral("C++"), QStringLiteral("cpp")},
        {QStringLiteral("Python"), QStringLiteral("python")}};
    for (const auto &item : exportButtons) {
        auto *button = new QPushButton(QStringLiteral("导出 ") + item.first, this);
        exports->addWidget(button);
        connect(button, &QPushButton::clicked, this, [this, kind = item.second] { emit exportRequested(kind); });
    }
    exports->addStretch();
    layout->addLayout(exports);

    m_model = new ResultTableModel(this);
    m_table = new QTableView(this);
    m_table->setModel(m_model);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setStretchLastSection(true);
    layout->addWidget(m_table, 2);
    connect(m_table, &QTableView::clicked, this, &CalibrationResultPage::onResultClicked);

    auto *details = new QHBoxLayout;
    m_reliability = new QLabel(QStringLiteral("可靠性报告将在计算后显示。"), this);
    m_reliability->setWordWrap(true);
    details->addWidget(m_reliability, 1);
    m_matrix = new QPlainTextEdit(this);
    m_matrix->setObjectName(QStringLiteral("resultMatrix"));
    m_matrix->setReadOnly(true);
    m_matrix->setPlaceholderText(QStringLiteral("camera→gripper 4×4 矩阵"));
    details->addWidget(m_matrix, 1);
    layout->addLayout(details, 1);

    m_log = new QPlainTextEdit(this);
    m_log->setReadOnly(true);
    m_log->setMaximumHeight(130);
    layout->addWidget(m_log);
}

CalibrationResult CalibrationResultPage::selectedResult() const
{
    const QModelIndexList selected = m_table->selectionModel()->selectedRows();
    if (!selected.isEmpty()) return m_model->resultAt(selected.first().row());
    return m_model->rowCount() > 0 ? m_model->resultAt(0) : CalibrationResult{};
}

void CalibrationResultPage::setResults(const QVector<CalibrationResult> &results)
{
    m_model->setResults(results);
    for (int index = 0; index < results.size(); ++index) {
        if (results.at(index).recommended) {
            m_table->selectRow(index);
            onResultClicked(m_model->index(index, 0));
            return;
        }
    }
    if (!results.isEmpty()) {
        m_table->selectRow(0);
        onResultClicked(m_model->index(0, 0));
    }
}

void CalibrationResultPage::showReliability(const CalibrationResult &result)
{
    if (!result.success) {
        m_reliability->setText(result.message.isEmpty() ? QStringLiteral("尚未生成可靠性报告。") : result.message);
        return;
    }
    const ReliabilityReport &training = result.trainingReport;
    QString text = QStringLiteral("训练：旋转 RMSE %1° | 平移 RMSE %2 m\n平均：%3° / %4 m\n最大：%5° / %6 m\n异常样本：%7\n状态：%8")
                       .arg(training.rotationRmseDeg, 0, 'f', 5).arg(training.translationRmseM, 0, 'f', 7)
                       .arg(training.rotationMeanDeg, 0, 'f', 5).arg(training.translationMeanM, 0, 'f', 7)
                       .arg(training.rotationMaxDeg, 0, 'f', 5).arg(training.translationMaxM, 0, 'f', 7)
                       .arg(training.outlierCount)
                       .arg(training.passed ? QStringLiteral("通过") : QStringLiteral("未通过"));
    if (result.validationReport.available)
        text += QStringLiteral("\n验证：旋转 RMSE %1° | 平移 RMSE %2 m | %3")
                    .arg(result.validationReport.rotationRmseDeg, 0, 'f', 5)
                    .arg(result.validationReport.translationRmseM, 0, 'f', 7)
                    .arg(result.validationReport.passed ? QStringLiteral("通过") : QStringLiteral("未通过"));
    m_reliability->setText(text);
}

void CalibrationResultPage::showMatrix(const CalibrationResult &result)
{
    if (!result.success) {
        m_matrix->setPlainText(result.message);
        return;
    }
    QStringList lines;
    lines << QStringLiteral("%1 | camera→gripper").arg(methodName(result.method));
    for (const auto &row : result.cameraToGripper)
        lines << QStringLiteral("[%1, %2, %3, %4]")
                      .arg(row[0], 0, 'f', 10).arg(row[1], 0, 'f', 10)
                      .arg(row[2], 0, 'f', 10).arg(row[3], 0, 'f', 10);
    m_matrix->setPlainText(lines.join('\n'));
}

void CalibrationResultPage::appendLog(const QString &message)
{
    m_log->appendPlainText(message);
}

void CalibrationResultPage::clearResults()
{
    m_model->setResults({});
    m_reliability->setText(QStringLiteral("尚未生成可靠性报告。"));
    m_matrix->clear();
}

void CalibrationResultPage::onResultClicked(const QModelIndex &index)
{
    const CalibrationResult result = m_model->resultAt(index.row());
    showReliability(result);
    showMatrix(result);
}

} // namespace handeye
