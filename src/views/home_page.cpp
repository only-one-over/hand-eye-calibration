#include "views/home_page.h"

#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <QPair>
#include <QVector>

namespace handeye {

HomePage::HomePage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(28, 28, 28, 28);
    layout->setSpacing(18);

    auto *title = new QLabel(QStringLiteral("手眼标定工具"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(24);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *subtitle = new QLabel(QStringLiteral("按向导完成参数设置、数据上传、数据确认，最后查看标定结果与分析。"), this);
    subtitle->setObjectName(QStringLiteral("homeSubtitle"));
    layout->addWidget(subtitle);

    auto *startButton = new QPushButton(QStringLiteral("开始手眼标定"), this);
    startButton->setObjectName(QStringLiteral("startCalibrationButton"));
    startButton->setProperty("variant", "primary");
    startButton->setMinimumHeight(44);
    layout->addWidget(startButton);
    connect(startButton, &QPushButton::clicked, this, &HomePage::startCalibrationRequested);

    auto *steps = new QGridLayout;
    steps->setHorizontalSpacing(14);
    const QStringList stepTitles = {QStringLiteral("1  设置参数"), QStringLiteral("2  上传本轮数据"),
                                    QStringLiteral("3  确认当前数据"), QStringLiteral("4  查看结果分析")};
    const QStringList stepTexts = {
        QStringLiteral("选择算法、姿态格式、单位、棋盘格和相机内参。"),
        QStringLiteral("上传机器人坐标和同顺序的标定板图片。"),
        QStringLiteral("逐组检查 TCP、图片和 target→camera 是否正确对应。"),
        QStringLiteral("查看矩阵、误差、质量评分和可靠性分析。")};
    for (int index = 0; index < stepTitles.size(); ++index) {
        auto *card = new QGroupBox(stepTitles.at(index), this);
        auto *cardLayout = new QVBoxLayout(card);
        auto *text = new QLabel(stepTexts.at(index), card);
        text->setWordWrap(true);
        cardLayout->addWidget(text);
        auto *stepHint = new QLabel(QStringLiteral("完成后点击页面底部的“下一步”。"), card);
        stepHint->setStyleSheet(QStringLiteral("color: #667085;"));
        cardLayout->addWidget(stepHint);
        steps->addWidget(card, 0, index);
    }
    layout->addLayout(steps);

    auto *stateGroup = new QGroupBox(QStringLiteral("当前批次状态"), this);
    auto *stateLayout = new QVBoxLayout(stateGroup);
    m_sampleSummary = new QLabel(QStringLiteral("机器人坐标：0 组 | 标定图片：0 张 | target→camera：未生成 | 结果：未计算"),
                                  stateGroup);
    m_sampleSummary->setWordWrap(true);
    stateLayout->addWidget(m_sampleSummary);
    m_statusLabel = new QLabel(QStringLiteral("准备开始。"), stateGroup);
    m_statusLabel->setWordWrap(true);
    stateLayout->addWidget(m_statusLabel);
    layout->addWidget(stateGroup);

    auto *advancedGroup = new QGroupBox(QStringLiteral("高级入口（不影响主流程）"), this);
    auto *quickLayout = new QGridLayout(advancedGroup);
    const QVector<QPair<QString, int>> quickActions = {
        {QStringLiteral("自主标定相机"), 3}, {QStringLiteral("手动输入位姿"), 4},
        {QStringLiteral("生成标定板 PDF"), 5}, {QStringLiteral("查看标定结果"), 7}};
    for (int index = 0; index < quickActions.size(); ++index) {
        auto *button = new QPushButton(quickActions.at(index).first, advancedGroup);
        quickLayout->addWidget(button, index / 2, index % 2);
        connect(button, &QPushButton::clicked, this, [this, page = quickActions.at(index).second] {
            emit navigateRequested(page);
        });
    }
    layout->addWidget(advancedGroup);
    layout->addStretch();
}

void HomePage::setSummary(int sampleCount, int imageCount, int targetPoseCount, bool hasResults)
{
    m_sampleSummary->setText(QStringLiteral("机器人坐标：%1 组 | 标定图片：%2 张 | target→camera：%3 组 | 结果：%4")
                                 .arg(sampleCount).arg(imageCount).arg(targetPoseCount)
                                 .arg(hasResults ? QStringLiteral("已计算") : QStringLiteral("未计算")));
}

void HomePage::setStatus(const QString &status)
{
    m_statusLabel->setText(status);
}

} // namespace handeye
