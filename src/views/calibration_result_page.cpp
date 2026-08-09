#include "views/calibration_result_page.h"

#include "models/calibration_session_model.h"

#include <QHBoxLayout>
#include <QAbstractItemView>
#include <QFont>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTableWidgetItem>
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
    auto *fixedTarget = new QPushButton(QStringLiteral("计算 fixed target pose"), this);
    fixedTarget->setObjectName(QStringLiteral("computeFixedTargetButton"));
    auto *optimize = new QPushButton(QStringLiteral("非线性优化精修"), this);
    optimize->setObjectName(QStringLiteral("optimizeRecommendedButton"));
    optimize->setProperty("variant", "primary");
    auto *importValidation = new QPushButton(QStringLiteral("导入独立验证数据"), this);
    m_referenceSample = new QSpinBox(this);
    m_referenceSample->setObjectName(QStringLiteral("fixedTargetReferenceSample"));
    m_referenceSample->setRange(-1, 999999);
    m_referenceSample->setValue(-1);
    m_referenceSample->setSpecialValueText(QStringLiteral("鲁棒均值"));
    actions->addWidget(new QLabel(QStringLiteral("reference ID"), this));
    actions->addWidget(m_referenceSample);
    actions->addWidget(calculateSelected);
    actions->addWidget(calculateAll);
    actions->addWidget(fixedTarget);
    actions->addWidget(optimize);
    actions->addWidget(importValidation);
    actions->addStretch();
    layout->addLayout(actions);
    connect(calculateSelected, &QPushButton::clicked, this, &CalibrationResultPage::calculateSelectedRequested);
    connect(calculateAll, &QPushButton::clicked, this, &CalibrationResultPage::calculateAllRequested);
    connect(fixedTarget, &QPushButton::clicked, this, [this] {
        emit computeFixedTargetRequested(m_referenceSample->value());
    });
    connect(optimize, &QPushButton::clicked, this, &CalibrationResultPage::optimizeRequested);
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

    m_poseReportTable = new QTableWidget(this);
    m_poseReportTable->setObjectName(QStringLiteral("fixedTargetReportTable"));
    m_poseReportTable->setColumnCount(6);
    m_poseReportTable->setHorizontalHeaderLabels({QStringLiteral("样本 ID"), QStringLiteral("预测 X/Y/Z"),
                                                   QStringLiteral("旋转误差(°)"), QStringLiteral("平移误差(m)"),
                                                   QStringLiteral("reference 旋转(°)"), QStringLiteral("异常")});
    m_poseReportTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_poseReportTable->setMaximumHeight(190);
    layout->addWidget(m_poseReportTable);

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

void CalibrationResultPage::setReferenceSampleIds(const QVector<PoseSample> &samples)
{
    if (!m_referenceSample) return;
    const int previous = m_referenceSample->value();
    m_referenceSample->setRange(-1, 999999);
    m_referenceSample->setValue(-1);
    if (previous > 0) {
        for (const PoseSample &sample : samples) {
            if (sample.id == previous) {
                m_referenceSample->setValue(previous);
                break;
            }
        }
    }
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
    QString text = QStringLiteral("输入模式：%1\n训练：旋转 RMSE %2° | 平移 RMSE %3 m\n平均：%4° / %5 m\n最大：%6° / %7 m\n异常样本：%8\n状态：%9")
                       .arg(result.fixedPointReport.available ? QStringLiteral("FixedPoint3D")
                                                               : QStringLiteral("PosePairs"))
                       .arg(training.rotationRmseDeg, 0, 'f', 5).arg(training.translationRmseM, 0, 'f', 7)
                       .arg(training.rotationMeanDeg, 0, 'f', 5).arg(training.translationMeanM, 0, 'f', 7)
                       .arg(training.rotationMaxDeg, 0, 'f', 5).arg(training.translationMaxM, 0, 'f', 7)
                       .arg(training.outlierCount)
                       .arg(training.passed ? QStringLiteral("通过") : QStringLiteral("未通过"));
    if (result.fixedPointReport.available) {
        text += QStringLiteral("\n固定点：RMSE %1 m | 平均 %2 m | 最大 %3 m | 异常 %4")
                    .arg(result.fixedPointReport.rmseM, 0, 'f', 6)
                    .arg(result.fixedPointReport.meanErrorM, 0, 'f', 6)
                    .arg(result.fixedPointReport.maxErrorM, 0, 'f', 6)
                    .arg(result.fixedPointReport.outlierCount);
    }
    if (result.fixedTargetReport.available) {
        text += QStringLiteral("\nFixed target：旋转 RMSE %1° | 平移 RMSE %2 m | 异常 %3")
                    .arg(result.fixedTargetReport.rotationRmseDeg, 0, 'f', 5)
                    .arg(result.fixedTargetReport.translationRmseM, 0, 'f', 6)
                    .arg(result.fixedTargetReport.outlierCount);
    }
    if (result.qualityReport.available) {
        text += QStringLiteral("\nPose Quality Score：%1/100（%2）\n样本 %3/25 | 旋转幅度 %4/25 | 旋转轴 %5/25 | 空间分布 %6/25")
                    .arg(result.qualityReport.totalScore).arg(result.qualityReport.level)
                    .arg(result.qualityReport.sampleScore).arg(result.qualityReport.rotationAmplitudeScore)
                    .arg(result.qualityReport.rotationAxisScore).arg(result.qualityReport.spatialDistributionScore);
    }
    if (result.optimizationReport.available) {
        text += QStringLiteral("\n优化：%1 → %2，迭代 %3 次，%4")
                    .arg(result.optimizationReport.beforeTranslationRmseM, 0, 'f', 6)
                    .arg(result.optimizationReport.afterTranslationRmseM, 0, 'f', 6)
                    .arg(result.optimizationReport.iterations)
                    .arg(result.optimizationReport.converged ? QStringLiteral("已收敛") : QStringLiteral("达到停止条件"));
    }
    m_poseReportTable->clearContents();
    if (result.fixedTargetReport.available) {
        m_poseReportTable->setHorizontalHeaderLabels({QStringLiteral("样本 ID"), QStringLiteral("预测 X/Y/Z"),
                                                       QStringLiteral("旋转误差(°)"), QStringLiteral("平移误差(m)"),
                                                       QStringLiteral("reference 旋转(°)"), QStringLiteral("异常")});
        m_poseReportTable->setRowCount(result.fixedTargetReport.samples.size());
        for (int row = 0; row < result.fixedTargetReport.samples.size(); ++row) {
            const FixedTargetPoseSample &sample = result.fixedTargetReport.samples.at(row);
            const Vector3 &translation = sample.predictedTranslation;
            m_poseReportTable->setItem(row, 0, new QTableWidgetItem(QString::number(sample.sampleId)));
            m_poseReportTable->setItem(row, 1, new QTableWidgetItem(
                                                 QStringLiteral("[%1, %2, %3]").arg(translation[0], 0, 'f', 5)
                                                     .arg(translation[1], 0, 'f', 5).arg(translation[2], 0, 'f', 5)));
            m_poseReportTable->setItem(row, 2, new QTableWidgetItem(QString::number(sample.rotationErrorToMeanDeg, 'f', 4)));
            m_poseReportTable->setItem(row, 3, new QTableWidgetItem(QString::number(sample.translationErrorToMeanM, 'f', 6)));
            m_poseReportTable->setItem(row, 4, new QTableWidgetItem(QString::number(sample.rotationErrorToReferenceDeg, 'f', 4)));
            m_poseReportTable->setItem(row, 5, new QTableWidgetItem(sample.outlier ? QStringLiteral("异常") : QStringLiteral("正常")));
        }
    } else if (result.fixedPointReport.available) {
        m_poseReportTable->setHorizontalHeaderLabels(QStringList{
            QStringLiteral("样本 ID"), QStringLiteral("预测 base XYZ"), QStringLiteral("残差(m)"),
            QStringLiteral("异常"), QString(), QString()});
        m_poseReportTable->setRowCount(result.fixedPointReport.samples.size());
        for (int row = 0; row < result.fixedPointReport.samples.size(); ++row) {
            const FixedPointSample &sample = result.fixedPointReport.samples.at(row);
            const Vector3 &point = sample.predictedBasePoint;
            m_poseReportTable->setItem(row, 0, new QTableWidgetItem(QString::number(sample.sampleId)));
            m_poseReportTable->setItem(row, 1, new QTableWidgetItem(
                                                 QStringLiteral("[%1, %2, %3]").arg(point[0], 0, 'f', 5)
                                                     .arg(point[1], 0, 'f', 5).arg(point[2], 0, 'f', 5)));
            m_poseReportTable->setItem(row, 2, new QTableWidgetItem(QString::number(sample.residualM, 'f', 6)));
            m_poseReportTable->setItem(row, 3, new QTableWidgetItem(sample.outlier ? QStringLiteral("异常") : QStringLiteral("正常")));
        }
    } else {
        m_poseReportTable->setRowCount(0);
    }
    m_poseReportTable->resizeColumnsToContents();
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
