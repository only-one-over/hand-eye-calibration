#include "views/camera_calibration_page.h"

#include <QAbstractItemView>
#include <QColor>
#include <QFont>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QResizeEvent>
#include <QStringList>
#include <QTableWidget>
#include <QVBoxLayout>

namespace handeye {

CameraCalibrationPage::CameraCalibrationPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(18, 18, 18, 18);
    layout->setSpacing(12);

    auto *title = new QLabel(QStringLiteral("自主相机内参标定"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    m_boardSummary = new QLabel(this);
    m_boardSummary->setWordWrap(true);
    layout->addWidget(m_boardSummary);

    auto *actions = new QHBoxLayout;
    auto *selectButton = new QPushButton(QStringLiteral("选择标定图片"), this);
    selectButton->setObjectName(QStringLiteral("selectCameraCalibrationImagesButton"));
    auto *detectButton = new QPushButton(QStringLiteral("检测棋盘格角点"), this);
    detectButton->setObjectName(QStringLiteral("detectCameraCalibrationButton"));
    auto *calibrateButton = new QPushButton(QStringLiteral("开始相机标定"), this);
    calibrateButton->setObjectName(QStringLiteral("calibrateCameraButton"));
    calibrateButton->setProperty("variant", "primary");
    auto *applyButton = new QPushButton(QStringLiteral("应用内参"), this);
    applyButton->setObjectName(QStringLiteral("applyCameraIntrinsicsButton"));
    auto *clearButton = new QPushButton(QStringLiteral("清空图片"), this);
    clearButton->setObjectName(QStringLiteral("clearCameraCalibrationButton"));
    auto *parameterButton = new QPushButton(QStringLiteral("前往参数页"), this);
    actions->addWidget(selectButton);
    actions->addWidget(detectButton);
    actions->addWidget(calibrateButton);
    actions->addWidget(applyButton);
    actions->addWidget(clearButton);
    actions->addWidget(parameterButton);
    actions->addStretch();
    layout->addLayout(actions);
    connect(selectButton, &QPushButton::clicked, this, &CameraCalibrationPage::selectImagesRequested);
    connect(detectButton, &QPushButton::clicked, this, &CameraCalibrationPage::detectRequested);
    connect(calibrateButton, &QPushButton::clicked, this, &CameraCalibrationPage::calibrateRequested);
    connect(applyButton, &QPushButton::clicked, this, &CameraCalibrationPage::applyRequested);
    connect(clearButton, &QPushButton::clicked, this, &CameraCalibrationPage::clearRequested);
    connect(parameterButton, &QPushButton::clicked, this, &CameraCalibrationPage::goParametersRequested);

    m_status = new QLabel(QStringLiteral("请选择 10～30 张不同姿态的棋盘格图片，至少需要 6 张成功检测。"), this);
    m_status->setWordWrap(true);
    layout->addWidget(m_status);

    auto *splitterLayout = new QHBoxLayout;
    m_table = new QTableWidget(this);
    m_table->setObjectName(QStringLiteral("cameraCalibrationTable"));
    m_table->setColumnCount(7);
    m_table->setHorizontalHeaderLabels({QStringLiteral("图片路径"), QStringLiteral("分辨率"),
                                        QStringLiteral("角点状态"), QStringLiteral("角点数"),
                                        QStringLiteral("单图 RMSE(px)"), QStringLiteral("参与计算"),
                                        QStringLiteral("备注")});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(6, QHeaderView::Stretch);
    connect(m_table, &QTableWidget::itemSelectionChanged, this, &CameraCalibrationPage::onCurrentRowChanged);
    splitterLayout->addWidget(m_table, 3);

    auto *previewPanel = new QWidget(this);
    auto *previewLayout = new QVBoxLayout(previewPanel);
    previewLayout->addWidget(new QLabel(QStringLiteral("图片预览与角点"), previewPanel));
    m_preview = new QLabel(QStringLiteral("请选择图片"), previewPanel);
    m_preview->setAlignment(Qt::AlignCenter);
    m_preview->setMinimumSize(300, 240);
    m_preview->setStyleSheet(QStringLiteral("border: 1px solid #E0E0E0; background: #FAFAFA;"));
    previewLayout->addWidget(m_preview, 1);
    m_sampleInfo = new QLabel(QStringLiteral("图片路径：\n状态："), previewPanel);
    m_sampleInfo->setWordWrap(true);
    previewLayout->addWidget(m_sampleInfo);
    splitterLayout->addWidget(previewPanel, 2);
    layout->addLayout(splitterLayout, 2);

    auto *resultLayout = new QHBoxLayout;
    m_metrics = new QLabel(QStringLiteral("标定结果将在计算后显示。"), this);
    m_metrics->setWordWrap(true);
    resultLayout->addWidget(m_metrics, 2);
    m_matrix = new QPlainTextEdit(this);
    m_matrix->setObjectName(QStringLiteral("cameraCalibrationMatrix"));
    m_matrix->setReadOnly(true);
    m_matrix->setPlaceholderText(QStringLiteral("相机内参矩阵 K"));
    resultLayout->addWidget(m_matrix, 1);
    m_distortion = new QPlainTextEdit(this);
    m_distortion->setObjectName(QStringLiteral("cameraCalibrationDistortion"));
    m_distortion->setReadOnly(true);
    m_distortion->setPlaceholderText(QStringLiteral("k1,k2,p1,p2,k3"));
    resultLayout->addWidget(m_distortion, 1);
    layout->addLayout(resultLayout, 1);

    setBoardSpec(m_boardSpec);
    updateActions();
}

void CameraCalibrationPage::setBoardSpec(const BoardSpec &board)
{
    m_boardSpec = board;
    m_boardSummary->setText(QStringLiteral("当前棋盘格：%1 × %2 个内角点，方格尺寸 %3 mm；普通针孔模型，自动估计 5 个畸变参数。")
                               .arg(board.innerCornersX).arg(board.innerCornersY)
                               .arg(board.squareSizeM * 1000.0, 0, 'f', 3));
}

void CameraCalibrationPage::setReport(const CameraCalibrationReport &report)
{
    m_report = report;
    m_table->setRowCount(0);
    for (int row = 0; row < report.samples.size(); ++row) {
        const CameraCalibrationSample &sample = report.samples.at(row);
        m_table->insertRow(row);
        const QString resolution = sample.imageWidth > 0
                                       ? QStringLiteral("%1 × %2").arg(sample.imageWidth).arg(sample.imageHeight)
                                       : QStringLiteral("-");
        const QString rmse = sample.reprojectionRmsePx > 0.0
                                 ? QString::number(sample.reprojectionRmsePx, 'f', 3) : QStringLiteral("-");
        const QStringList values = {sample.imagePath, resolution,
                                    cameraCalibrationSampleStatusName(sample.status),
                                    QString::number(sample.detectedCornerCount), rmse,
                                    sample.used ? QStringLiteral("是") : QStringLiteral("否"), sample.message};
        for (int column = 0; column < values.size(); ++column)
            m_table->setItem(row, column, new QTableWidgetItem(values.at(column)));
        if (sample.outlier) {
            for (int column = 0; column < m_table->columnCount(); ++column)
                m_table->item(row, column)->setBackground(QColor(QStringLiteral("#FCEBEA")));
        }
    }
    if (m_table->rowCount() > 0) m_table->selectRow(0);

    if (!report.message.isEmpty()) m_status->setText(report.message);
    if (!report.success) {
        m_metrics->setText(report.errors.isEmpty() ? QStringLiteral("标定结果将在计算后显示。")
                                                   : report.errors.join('\n'));
        m_matrix->clear();
        m_distortion->clear();
    } else {
        QString metrics = QStringLiteral("状态：%1\n有效图片：%2/%3\n剔除图片：%4\n总体 RMS：%5 px\n平均单图 RMSE：%6 px\n最大单图 RMSE：%7 px\n覆盖性：%8")
                              .arg(report.passed ? QStringLiteral("通过") : QStringLiteral("未通过"))
                              .arg(report.finalUsedCount).arg(report.initialImageCount).arg(report.outlierCount)
                              .arg(report.rmsPx, 0, 'f', 4).arg(report.meanRmsePx, 0, 'f', 4)
                              .arg(report.maxRmsePx, 0, 'f', 4)
                              .arg(report.coverageWarning ? QStringLiteral("有警告") : QStringLiteral("良好"));
        if (!report.warnings.isEmpty()) metrics += QStringLiteral("\n警告：") + report.warnings.join(QStringLiteral("；"));
        m_metrics->setText(metrics);
        m_matrix->setPlainText(QStringLiteral("K =\n%1").arg(formatMatrix(report.intrinsics.cameraMatrix)));
        m_distortion->setPlainText(QStringLiteral("[k1, k2, p1, p2, k3]\n[%1, %2, %3, %4, %5]")
                                       .arg(report.intrinsics.distortionCoeffs[0], 0, 'g', 12)
                                       .arg(report.intrinsics.distortionCoeffs[1], 0, 'g', 12)
                                       .arg(report.intrinsics.distortionCoeffs[2], 0, 'g', 12)
                                       .arg(report.intrinsics.distortionCoeffs[3], 0, 'g', 12)
                                       .arg(report.intrinsics.distortionCoeffs[4], 0, 'g', 12));
    }
    updateActions();
}

void CameraCalibrationPage::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    updatePreview();
}

void CameraCalibrationPage::onCurrentRowChanged()
{
    const QList<QTableWidgetItem *> selected = m_table->selectedItems();
    if (!selected.isEmpty()) showSample(selected.first()->row());
}

void CameraCalibrationPage::showSample(int row)
{
    if (row < 0 || row >= m_report.samples.size()) return;
    const CameraCalibrationSample &sample = m_report.samples.at(row);
    m_currentPixmap = QPixmap(sample.imagePath);
    if (!m_currentPixmap.isNull() && !sample.corners.isEmpty()) {
        QPainter painter(&m_currentPixmap);
        painter.setPen(QPen(QColor(QStringLiteral("#E8463A")), 3));
        for (const Vector2 &corner : sample.corners)
            painter.drawEllipse(QPointF(corner[0], corner[1]), 3.0, 3.0);
    }
    m_sampleInfo->setText(QStringLiteral("图片路径：%1\n状态：%2\n角点数：%3\n单图 RMSE：%4 px")
                             .arg(sample.imagePath, cameraCalibrationSampleStatusName(sample.status))
                             .arg(sample.detectedCornerCount)
                             .arg(sample.reprojectionRmsePx, 0, 'f', 4));
    updatePreview();
}

void CameraCalibrationPage::updatePreview()
{
    if (m_currentPixmap.isNull()) {
        m_preview->setPixmap({});
        if (m_report.samples.isEmpty()) m_preview->setText(QStringLiteral("请选择图片"));
        return;
    }
    m_preview->setText({});
    m_preview->setPixmap(m_currentPixmap.scaled(m_preview->size(), Qt::KeepAspectRatio,
                                                 Qt::SmoothTransformation));
}

void CameraCalibrationPage::updateActions()
{
    const bool hasImages = !m_report.samples.isEmpty();
    const bool enoughDetected = m_report.initialDetectedCount >= 6;
    for (QPushButton *button : findChildren<QPushButton *>()) {
        if (button->objectName() == QStringLiteral("detectCameraCalibrationButton")) button->setEnabled(hasImages);
        if (button->objectName() == QStringLiteral("calibrateCameraButton")) button->setEnabled(hasImages && enoughDetected);
        if (button->objectName() == QStringLiteral("applyCameraIntrinsicsButton"))
            button->setEnabled(m_report.success && m_report.intrinsics.valid);
        if (button->objectName() == QStringLiteral("clearCameraCalibrationButton")) button->setEnabled(hasImages);
    }
}

QString CameraCalibrationPage::formatMatrix(const Matrix3 &matrix)
{
    QStringList rows;
    for (const auto &row : matrix)
        rows.append(QStringLiteral("[%1, %2, %3]").arg(row[0], 0, 'g', 12)
                        .arg(row[1], 0, 'g', 12).arg(row[2], 0, 'g', 12));
    return rows.join('\n');
}

} // namespace handeye
