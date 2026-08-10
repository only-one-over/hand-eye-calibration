#include "views/capture_page.h"

#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace handeye {

CapturePage::CapturePage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(24, 24, 24, 24);
    layout->setSpacing(16);

    auto *title = new QLabel(QStringLiteral("本轮数据采集"), this);
    QFont font = title->font();
    font.setPointSize(20);
    font.setBold(true);
    title->setFont(font);
    layout->addWidget(title);

    auto *warning = new QLabel(
        QStringLiteral("机器人坐标和标定板图片必须来自同一组采集动作。请按相同编号和顺序上传，程序不会自动猜测配对关系。"),
        this);
    warning->setWordWrap(true);
    layout->addWidget(warning);

    auto *uploadGroup = new QGroupBox(QStringLiteral("上传本轮数据"), this);
    auto *uploadLayout = new QVBoxLayout(uploadGroup);
    auto *robotButton = new QPushButton(QStringLiteral("上传机器人坐标"), uploadGroup);
    robotButton->setObjectName(QStringLiteral("uploadRobotButton"));
    auto *imageButton = new QPushButton(QStringLiteral("上传标定板图片"), uploadGroup);
    imageButton->setObjectName(QStringLiteral("uploadImagesButton"));
    uploadLayout->addWidget(robotButton);
    uploadLayout->addWidget(imageButton);
    connect(robotButton, &QPushButton::clicked, this, &CapturePage::uploadRobotRequested);
    connect(imageButton, &QPushButton::clicked, this, &CapturePage::uploadImagesRequested);
    layout->addWidget(uploadGroup);

    auto *generateBoardButton = new QPushButton(QStringLiteral("生成当前参数的标定板 PDF"), this);
    generateBoardButton->setObjectName(QStringLiteral("captureGenerateBoardPdfButton"));
    layout->addWidget(generateBoardButton);
    connect(generateBoardButton, &QPushButton::clicked, this, &CapturePage::generateBoardRequested);

    auto *statusGroup = new QGroupBox(QStringLiteral("采集进度"), this);
    auto *statusLayout = new QVBoxLayout(statusGroup);
    m_summary = new QLabel(QStringLiteral("机器人坐标：0 组 | 标定图片：0 张 | target→camera：0 组"), statusGroup);
    m_summary->setWordWrap(true);
    statusLayout->addWidget(m_summary);
    m_processingStatus = new QLabel(QStringLiteral("尚未处理标定板图片。"), statusGroup);
    m_processingStatus->setWordWrap(true);
    statusLayout->addWidget(m_processingStatus);
    layout->addWidget(statusGroup);

    auto *processButton = new QPushButton(QStringLiteral("处理标定板图片并生成 target→camera"), this);
    processButton->setObjectName(QStringLiteral("processBoardImagesButton"));
    processButton->setProperty("variant", "primary");
    layout->addWidget(processButton);
    connect(processButton, &QPushButton::clicked, this, &CapturePage::processRequested);

    auto *flowBar = new QHBoxLayout;
    auto *flowHint = new QLabel(QStringLiteral("上传完成后点击下一步。若使用图片数据，程序会在进入数据页前尝试处理棋盘格。"), this);
    flowHint->setWordWrap(true);
    flowHint->setStyleSheet(QStringLiteral("color: #667085;"));
    flowBar->addWidget(flowHint, 1);
    auto *nextButton = new QPushButton(QStringLiteral("下一步：查看当前数据"), this);
    nextButton->setObjectName(QStringLiteral("captureNextButton"));
    nextButton->setProperty("variant", "primary");
    flowBar->addWidget(nextButton);
    layout->addLayout(flowBar);
    connect(nextButton, &QPushButton::clicked, this, &CapturePage::nextRequested);
    layout->addStretch();
}

void CapturePage::setSummary(int sampleCount, int imageCount, int targetPoseCount)
{
    m_summary->setText(QStringLiteral("机器人坐标：%1 组 | 标定图片：%2 张 | target→camera：%3 组")
                          .arg(sampleCount).arg(imageCount).arg(targetPoseCount));
}

void CapturePage::setProcessingResult(int processed, int succeeded)
{
    m_processingStatus->setText(QStringLiteral("图片处理完成：%1/%2 组成功生成 target→camera。")
                                    .arg(succeeded).arg(processed));
}

} // namespace handeye
