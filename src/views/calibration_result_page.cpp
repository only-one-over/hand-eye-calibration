#include "views/calibration_result_page.h"

#include "models/calibration_session_model.h"

#include <QHBoxLayout>
#include <QAbstractItemView>
#include <QFont>
#include <QHeaderView>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QDoubleSpinBox>
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
    m_calculateAllButton = new QPushButton(QStringLiteral("五种算法自动比较并推荐"), this);
    m_calculateAllButton->setObjectName(QStringLiteral("calculateAllButton"));
    m_calculateAllButton->setProperty("variant", "primary");
    m_fixedTargetButton = new QPushButton(QStringLiteral("计算 fixed target pose"), this);
    m_fixedTargetButton->setObjectName(QStringLiteral("computeFixedTargetButton"));
    auto *optimize = new QPushButton(QStringLiteral("非线性优化精修"), this);
    optimize->setObjectName(QStringLiteral("optimizeRecommendedButton"));
    optimize->setProperty("variant", "primary");
    auto *pipeline = new QPushButton(QStringLiteral("执行完整可靠性流水线"), this);
    pipeline->setObjectName(QStringLiteral("runReliabilityPipelineButton"));
    pipeline->setProperty("variant", "primary");
    auto *importValidation = new QPushButton(QStringLiteral("导入独立验证数据"), this);
    m_referenceSample = new QSpinBox(this);
    m_referenceSample->setObjectName(QStringLiteral("fixedTargetReferenceSample"));
    m_referenceSample->setRange(-1, 999999);
    m_referenceSample->setValue(-1);
    m_referenceSample->setSpecialValueText(QStringLiteral("鲁棒均值"));
    m_bootstrapResamples = new QSpinBox(this);
    m_bootstrapResamples->setObjectName(QStringLiteral("bootstrapResamples"));
    m_bootstrapResamples->setRange(20, 5000);
    m_bootstrapResamples->setValue(200);
    m_bootstrapConfidence = new QDoubleSpinBox(this);
    m_bootstrapConfidence->setObjectName(QStringLiteral("bootstrapConfidence"));
    m_bootstrapConfidence->setRange(0.50, 0.999);
    m_bootstrapConfidence->setSingleStep(0.01);
    m_bootstrapConfidence->setDecimals(3);
    m_bootstrapConfidence->setValue(0.95);
    actions->addWidget(new QLabel(QStringLiteral("reference ID"), this));
    actions->addWidget(m_referenceSample);
    actions->addWidget(calculateSelected);
    actions->addWidget(m_calculateAllButton);
    actions->addWidget(m_fixedTargetButton);
    actions->addWidget(optimize);
    actions->addWidget(pipeline);
    actions->addWidget(new QLabel(QStringLiteral("Bootstrap 次数"), this));
    actions->addWidget(m_bootstrapResamples);
    actions->addWidget(new QLabel(QStringLiteral("置信度"), this));
    actions->addWidget(m_bootstrapConfidence);
    actions->addWidget(importValidation);
    actions->addStretch();
    layout->addLayout(actions);
    connect(calculateSelected, &QPushButton::clicked, this, &CalibrationResultPage::calculateSelectedRequested);
    connect(m_calculateAllButton, &QPushButton::clicked, this, &CalibrationResultPage::calculateAllRequested);
    connect(m_fixedTargetButton, &QPushButton::clicked, this, [this] {
        emit computeFixedTargetRequested(m_referenceSample->value());
    });
    connect(optimize, &QPushButton::clicked, this, &CalibrationResultPage::optimizeRequested);
    connect(pipeline, &QPushButton::clicked, this, [this] {
        emit reliabilityPipelineRequested(m_bootstrapResamples->value(), m_bootstrapConfidence->value());
    });
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
    auto *reportColumn = new QVBoxLayout;
    m_axXbReport = new QLabel(QStringLiteral("AX=XB 一致性：尚未计算。"), this);
    m_axXbReport->setWordWrap(true);
    m_fixedTargetReport = new QLabel(QStringLiteral("Fixed Target 一致性：尚未计算。"), this);
    m_fixedTargetReport->setWordWrap(true);
    reportColumn->addWidget(m_axXbReport);
    reportColumn->addWidget(m_fixedTargetReport);
    reportColumn->addWidget(m_reliability);
    details->addLayout(reportColumn, 1);
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

    m_pipelineTable = new QTableWidget(this);
    m_pipelineTable->setObjectName(QStringLiteral("reliabilityPipelineTable"));
    m_pipelineTable->setColumnCount(3);
    m_pipelineTable->setHorizontalHeaderLabels({QStringLiteral("流水线阶段"), QStringLiteral("状态"),
                                                  QStringLiteral("说明")});
    m_pipelineTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_pipelineTable->setMaximumHeight(220);
    layout->addWidget(m_pipelineTable);
    m_uncertainty = new QLabel(QStringLiteral("Bootstrap 置信度将在完整流水线执行后显示。"), this);
    m_uncertainty->setWordWrap(true);
    m_uncertainty->setObjectName(QStringLiteral("bootstrapUncertainty"));
    layout->addWidget(m_uncertainty);

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
    const bool eyeToHand = !results.isEmpty()
                           && (results.first().eyeToHandPoseReport.available
                               || results.first().eyeToHandPointReport.available);
    if (m_calculateAllButton)
        m_calculateAllButton->setText(eyeToHand ? QStringLiteral("Shah / Li / 非线性自动比较")
                                                : QStringLiteral("五种算法自动比较并推荐"));
    if (m_fixedTargetButton) m_fixedTargetButton->setVisible(!eyeToHand);
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
        m_axXbReport->setText(QStringLiteral("AX=XB 一致性：尚未计算。"));
        m_fixedTargetReport->setText(QStringLiteral("Fixed Target 一致性：尚未计算。"));
        return;
    }
    if (result.eyeToHandPoseReport.available) {
        const EyeToHandPoseReport &report = result.eyeToHandPoseReport;
        m_axXbReport->setText(QStringLiteral("AX=YB 一致性（Eye-To-Hand）\n旋转 RMSE：%1° | 平移 RMSE：%2 m\n平均：%3° / %4 m | 最大：%5° / %6 m\n异常样本：%7 | 状态：%8")
                                  .arg(report.rotationRmseDeg, 0, 'f', 5)
                                  .arg(report.translationRmseM, 0, 'f', 7)
                                  .arg(report.rotationMeanDeg, 0, 'f', 5)
                                  .arg(report.translationMeanM, 0, 'f', 7)
                                  .arg(report.rotationMaxDeg, 0, 'f', 5)
                                  .arg(report.translationMaxM, 0, 'f', 7)
                                  .arg(report.outlierCount)
                                  .arg(report.success ? QStringLiteral("可用") : QStringLiteral("失败")));
        m_fixedTargetReport->setText(QStringLiteral("Eye-To-Hand 位姿一致性\n每组比较 T_base_target(robot) 与 T_base_target(camera)。\n输出：camera→base + target→gripper。"));
        m_poseReportTable->setColumnCount(4);
        m_poseReportTable->setHorizontalHeaderLabels({QStringLiteral("样本 ID"), QStringLiteral("旋转误差(°)"),
                                                       QStringLiteral("平移误差(m)"), QStringLiteral("异常")});
        m_poseReportTable->setRowCount(report.samples.size());
        for (int row = 0; row < report.samples.size(); ++row) {
            const EyeToHandPoseResidual &sample = report.samples.at(row);
            m_poseReportTable->setItem(row, 0, new QTableWidgetItem(QString::number(sample.sampleId)));
            m_poseReportTable->setItem(row, 1, new QTableWidgetItem(QString::number(sample.rotationErrorDeg, 'f', 5)));
            m_poseReportTable->setItem(row, 2, new QTableWidgetItem(QString::number(sample.translationErrorM, 'f', 7)));
            m_poseReportTable->setItem(row, 3, new QTableWidgetItem(sample.outlier ? QStringLiteral("异常") : QStringLiteral("正常")));
        }
        m_poseReportTable->resizeColumnsToContents();
        m_reliability->setText(QStringLiteral("Eye-To-Hand PosePairs | AX=YB RMSE：%1° / %2 m\n非线性优化：%3 → %4 m，%5")
                                   .arg(report.rotationRmseDeg, 0, 'f', 5)
                                   .arg(report.translationRmseM, 0, 'f', 7)
                                   .arg(result.optimizationReport.beforeTranslationRmseM, 0, 'f', 7)
                                   .arg(result.optimizationReport.afterTranslationRmseM, 0, 'f', 7)
                                   .arg(result.axXbReport.passed ? QStringLiteral("通过") : QStringLiteral("未通过")));
        return;
    }
    if (result.eyeToHandPointReport.available) {
        const EyeToHandPointReport &report = result.eyeToHandPointReport;
        m_axXbReport->setText(QStringLiteral("Eye-To-Hand 点基一致性\nRMSE：%1 m | 平均：%2 m | 最大：%3 m\n异常样本：%4\n线性诊断：rank %5/15 | condition number %6 | %7")
                                  .arg(report.rmseM, 0, 'f', 7).arg(report.meanErrorM, 0, 'f', 7)
                                  .arg(report.maxErrorM, 0, 'f', 7).arg(report.outlierCount)
                                  .arg(report.linearRank)
                                  .arg(report.linearConditionNumber, 0, 'g', 6)
                                  .arg(report.fullRank && report.conditionAcceptable ? QStringLiteral("通过")
                                                                                       : QStringLiteral("未通过")));
        m_fixedTargetReport->setText(QStringLiteral("输出：camera→base\nTCP 上特征点：[%1, %2, %3] m")
                                         .arg(report.pointInGripper[0], 0, 'f', 6)
                                         .arg(report.pointInGripper[1], 0, 'f', 6)
                                         .arg(report.pointInGripper[2], 0, 'f', 6));
        m_poseReportTable->setColumnCount(4);
        m_poseReportTable->setHorizontalHeaderLabels({QStringLiteral("样本 ID"), QStringLiteral("预测 base XYZ"),
                                                       QStringLiteral("残差(m)"), QStringLiteral("异常")});
        m_poseReportTable->setRowCount(report.samples.size());
        for (int row = 0; row < report.samples.size(); ++row) {
            const FixedPointSample &sample = report.samples.at(row);
            m_poseReportTable->setItem(row, 0, new QTableWidgetItem(QString::number(sample.sampleId)));
            m_poseReportTable->setItem(row, 1, new QTableWidgetItem(
                                                 QStringLiteral("[%1, %2, %3]").arg(sample.predictedBasePoint[0], 0, 'f', 5)
                                                     .arg(sample.predictedBasePoint[1], 0, 'f', 5)
                                                     .arg(sample.predictedBasePoint[2], 0, 'f', 5)));
            m_poseReportTable->setItem(row, 2, new QTableWidgetItem(QString::number(sample.residualM, 'f', 7)));
            m_poseReportTable->setItem(row, 3, new QTableWidgetItem(sample.outlier ? QStringLiteral("异常") : QStringLiteral("正常")));
        }
        m_poseReportTable->resizeColumnsToContents();
        m_reliability->setText(QStringLiteral("Eye-To-Hand FixedPoint3D | camera→base\n固定点残差 RMSE：%1 m | 非线性优化：%2 → %3 m\n线性系统：rank %4/15，condition number %5，%6")
                                   .arg(report.rmseM, 0, 'f', 7)
                                   .arg(result.optimizationReport.beforeTranslationRmseM, 0, 'f', 7)
                                   .arg(result.optimizationReport.afterTranslationRmseM, 0, 'f', 7)
                                   .arg(report.linearRank)
                                   .arg(report.linearConditionNumber, 0, 'g', 6)
                                   .arg(report.fullRank && report.conditionAcceptable ? QStringLiteral("诊断通过")
                                                                                         : QStringLiteral("诊断未通过")));
        return;
    }
    const AxXbReport &axXb = result.axXbReport;
    m_axXbReport->setText(QStringLiteral("AX=XB 一致性\n旋转 RMSE：%1° | 平移 RMSE：%2 m\n平均：%3° / %4 m | 最大：%5° / %6 m\n异常样本：%7 | 状态：%8")
                              .arg(axXb.rotationRmseDeg, 0, 'f', 5)
                              .arg(axXb.translationRmseM, 0, 'f', 7)
                              .arg(axXb.rotationMeanDeg, 0, 'f', 5)
                              .arg(axXb.translationMeanM, 0, 'f', 7)
                              .arg(axXb.rotationMaxDeg, 0, 'f', 5)
                              .arg(axXb.translationMaxM, 0, 'f', 7)
                              .arg(axXb.outlierCount)
                              .arg(axXb.passed ? QStringLiteral("通过") : QStringLiteral("未通过")));
    if (result.fixedTargetReport.available) {
        m_fixedTargetReport->setText(QStringLiteral("Fixed Target 一致性\n旋转 RMSE：%1° | 平移 RMSE：%2 m\n平均：%3° / %4 m | 最大：%5° / %6 m\n异常样本：%7")
                                         .arg(result.fixedTargetReport.rotationRmseDeg, 0, 'f', 5)
                                         .arg(result.fixedTargetReport.translationRmseM, 0, 'f', 7)
                                         .arg(result.fixedTargetReport.rotationMeanDeg, 0, 'f', 5)
                                         .arg(result.fixedTargetReport.translationMeanM, 0, 'f', 7)
                                         .arg(result.fixedTargetReport.rotationMaxDeg, 0, 'f', 5)
                                         .arg(result.fixedTargetReport.translationMaxM, 0, 'f', 7)
                                         .arg(result.fixedTargetReport.outlierCount));
    } else if (result.fixedPointReport.available) {
        m_fixedTargetReport->setText(QStringLiteral("FixedPoint3D 固定点一致性\nRMSE：%1 m | 平均：%2 m | 最大：%3 m\n异常样本：%4")
                                         .arg(result.fixedPointReport.rmseM, 0, 'f', 7)
                                         .arg(result.fixedPointReport.meanErrorM, 0, 'f', 7)
                                         .arg(result.fixedPointReport.maxErrorM, 0, 'f', 7)
                                         .arg(result.fixedPointReport.outlierCount));
    } else {
        m_fixedTargetReport->setText(QStringLiteral("Fixed Target 一致性：尚未计算。"));
    }
    QString text = QStringLiteral("输入模式：%1\n训练：旋转 RMSE %2° | 平移 RMSE %3 m\n平均：%4° / %5 m\n最大：%6° / %7 m\n异常样本：%8\n状态：%9")
                       .arg(result.fixedPointReport.available ? QStringLiteral("FixedPoint3D")
                                                               : QStringLiteral("PosePairs"))
                       .arg(axXb.rotationRmseDeg, 0, 'f', 5).arg(axXb.translationRmseM, 0, 'f', 7)
                       .arg(axXb.rotationMeanDeg, 0, 'f', 5).arg(axXb.translationMeanM, 0, 'f', 7)
                       .arg(axXb.rotationMaxDeg, 0, 'f', 5).arg(axXb.translationMaxM, 0, 'f', 7)
                       .arg(axXb.outlierCount)
                       .arg(axXb.passed ? QStringLiteral("通过") : QStringLiteral("未通过"));
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
    if (result.bootstrapReport.available) {
        text += QStringLiteral("\nBootstrap：%1/%2 成功，Bootstrap 成功率 %3%，旋转不确定度 %4°，平移不确定度 %5 m")
                    .arg(result.bootstrapReport.successfulResamples)
                    .arg(result.bootstrapReport.requestedResamples)
                    .arg(result.bootstrapReport.successRate * 100.0, 0, 'f', 1)
                    .arg(result.bootstrapReport.rotationNormStdDeg, 0, 'f', 5)
                    .arg(result.bootstrapReport.translationNormStdM, 0, 'f', 7);
        text += QStringLiteral("\nBootstrap 模式：%1 | 不确定度可靠：%2")
                    .arg(result.bootstrapReport.smallSampleMode ? QStringLiteral("小样本稳定模式")
                                                                : QStringLiteral("普通模式"))
                    .arg(result.bootstrapReport.uncertaintyReliable ? QStringLiteral("是") : QStringLiteral("否"));
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

void CalibrationResultPage::showPipelineReport(const ReliabilityPipelineReport &report)
{
    if (!m_pipelineTable || !m_uncertainty) return;
    m_pipelineTable->clearContents();
    m_pipelineTable->setRowCount(report.stages.size());
    for (int row = 0; row < report.stages.size(); ++row) {
        const PipelineStageReport &stage = report.stages.at(row);
        m_pipelineTable->setItem(row, 0, new QTableWidgetItem(stage.name));
        m_pipelineTable->setItem(row, 1, new QTableWidgetItem(pipelineStageStateName(stage.state)));
        m_pipelineTable->setItem(row, 2, new QTableWidgetItem(stage.message));
    }
    m_pipelineTable->resizeColumnsToContents();
    m_pipelineTable->horizontalHeader()->setStretchLastSection(true);
    const BootstrapReport &bootstrap = report.bootstrapReport;
    if (!bootstrap.available) {
        m_uncertainty->setText(QStringLiteral("Bootstrap 未执行：%1").arg(report.message));
        return;
    }
    const QString pipelineText =
        QStringLiteral("最终矩阵：%1 | 流水线：%2 | 样本 %3 → %4（自动剔除 %5）\n"
                       "Bootstrap：%6/%7 成功，Bootstrap 成功率 %8%\n"
                       "旋转标准差：[%9, %10, %11]°，95%% 区间：[%12, %13, %14]° ～ [%15, %16, %17]°\n"
                       "平移标准差：[%18, %19, %20] m，95%% 区间：[%21, %22, %23] m ～ [%24, %25, %26] m")
            .arg(methodName(report.finalMethod))
            .arg(report.passed ? QStringLiteral("通过") : QStringLiteral("有警告"))
            .arg(report.initialSampleCount).arg(report.finalSampleCount).arg(report.autoRemovedCount)
            .arg(bootstrap.successfulResamples).arg(bootstrap.requestedResamples)
            .arg(bootstrap.successRate * 100.0, 0, 'f', 1)
            .arg(bootstrap.rotationStdDeg[0], 0, 'f', 5)
            .arg(bootstrap.rotationStdDeg[1], 0, 'f', 5)
            .arg(bootstrap.rotationStdDeg[2], 0, 'f', 5)
            .arg(bootstrap.rotationLowerDeg[0], 0, 'f', 5)
            .arg(bootstrap.rotationLowerDeg[1], 0, 'f', 5)
            .arg(bootstrap.rotationLowerDeg[2], 0, 'f', 5)
            .arg(bootstrap.rotationUpperDeg[0], 0, 'f', 5)
            .arg(bootstrap.rotationUpperDeg[1], 0, 'f', 5)
            .arg(bootstrap.rotationUpperDeg[2], 0, 'f', 5)
            .arg(bootstrap.translationStdM[0], 0, 'f', 7)
            .arg(bootstrap.translationStdM[1], 0, 'f', 7)
            .arg(bootstrap.translationStdM[2], 0, 'f', 7)
            .arg(bootstrap.translationLowerM[0], 0, 'f', 7)
            .arg(bootstrap.translationLowerM[1], 0, 'f', 7)
            .arg(bootstrap.translationLowerM[2], 0, 'f', 7)
            .arg(bootstrap.translationUpperM[0], 0, 'f', 7)
            .arg(bootstrap.translationUpperM[1], 0, 'f', 7)
            .arg(bootstrap.translationUpperM[2], 0, 'f', 7);
    m_uncertainty->setText(pipelineText + QStringLiteral("\nBootstrap 模式：%1 | 不确定度可靠：%2")
                                             .arg(bootstrap.smallSampleMode ? QStringLiteral("小样本稳定模式")
                                                                         : QStringLiteral("普通模式"))
                                             .arg(bootstrap.uncertaintyReliable ? QStringLiteral("是")
                                                                                : QStringLiteral("否")));
}

void CalibrationResultPage::showMatrix(const CalibrationResult &result)
{
    if (!result.success) {
        m_matrix->setPlainText(result.message);
        return;
    }
    QStringList lines;
    const bool eyeToHand = result.eyeToHandPoseReport.available || result.eyeToHandPointReport.available;
    const Matrix4 &primary = eyeToHand ? result.cameraToBase : result.cameraToGripper;
    lines << QStringLiteral("%1 | %2").arg(methodName(result.method), eyeToHand ? QStringLiteral("camera→base") : QStringLiteral("camera→gripper"));
    for (const auto &row : primary)
        lines << QStringLiteral("[%1, %2, %3, %4]")
                      .arg(row[0], 0, 'f', 10).arg(row[1], 0, 'f', 10)
                      .arg(row[2], 0, 'f', 10).arg(row[3], 0, 'f', 10);
    if (eyeToHand && result.eyeToHandPoseReport.available) {
        lines << QStringLiteral("\ntarget→gripper");
        for (const auto &row : result.targetToGripper)
            lines << QStringLiteral("[%1, %2, %3, %4]")
                          .arg(row[0], 0, 'f', 10).arg(row[1], 0, 'f', 10)
                          .arg(row[2], 0, 'f', 10).arg(row[3], 0, 'f', 10);
    } else if (eyeToHand) {
        lines << QStringLiteral("\npoint in gripper: [%1, %2, %3] m")
                      .arg(result.pointInGripper[0], 0, 'f', 10)
                      .arg(result.pointInGripper[1], 0, 'f', 10)
                      .arg(result.pointInGripper[2], 0, 'f', 10);
    }
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
    if (m_axXbReport) m_axXbReport->setText(QStringLiteral("AX=XB 一致性：尚未计算。"));
    if (m_fixedTargetReport) m_fixedTargetReport->setText(QStringLiteral("Fixed Target 一致性：尚未计算。"));
    m_matrix->clear();
    if (m_pipelineTable) m_pipelineTable->setRowCount(0);
    if (m_uncertainty) m_uncertainty->setText(QStringLiteral("Bootstrap 置信度将在完整流水线执行后显示。"));
}

void CalibrationResultPage::onResultClicked(const QModelIndex &index)
{
    const CalibrationResult result = m_model->resultAt(index.row());
    showReliability(result);
    showMatrix(result);
}

} // namespace handeye
