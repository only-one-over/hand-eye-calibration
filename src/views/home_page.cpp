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

    auto *subtitle = new QLabel(QStringLiteral("从设置参数到得到 camera→gripper 矩阵，只需要三步。"), this);
    subtitle->setObjectName(QStringLiteral("homeSubtitle"));
    layout->addWidget(subtitle);

    auto *steps = new QGridLayout;
    steps->setHorizontalSpacing(14);
    const QStringList stepTitles = {QStringLiteral("1  设置参数"), QStringLiteral("2  上传本轮数据"),
                                    QStringLiteral("3  执行手眼标定")};
    const QStringList stepTexts = {
        QStringLiteral("选择算法、姿态格式、单位、棋盘格和相机内参。"),
        QStringLiteral("上传机器人坐标和同顺序的标定板图片。"),
        QStringLiteral("处理图片、比较五种算法并查看可靠性报告。")};
    const QVector<int> pages = {2, 1, 6};
    for (int index = 0; index < 3; ++index) {
        auto *card = new QGroupBox(stepTitles.at(index), this);
        auto *cardLayout = new QVBoxLayout(card);
        auto *text = new QLabel(stepTexts.at(index), card);
        text->setWordWrap(true);
        cardLayout->addWidget(text);
        auto *button = new QPushButton(QStringLiteral("开始"), card);
        cardLayout->addWidget(button);
        connect(button, &QPushButton::clicked, this, [this, page = pages.at(index)] {
            emit navigateRequested(page);
        });
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

    auto *quickLayout = new QGridLayout;
    const QVector<QPair<QString, int>> quickActions = {
        {QStringLiteral("设置参数"), 2}, {QStringLiteral("自主标定相机"), 3},
        {QStringLiteral("手动输入位姿"), 4}, {QStringLiteral("开始采集"), 1},
        {QStringLiteral("查看当前数据"), 5}, {QStringLiteral("查看标定结果"), 6}};
    for (int index = 0; index < quickActions.size(); ++index) {
        auto *button = new QPushButton(quickActions.at(index).first, this);
        quickLayout->addWidget(button, index / 2, index % 2);
        connect(button, &QPushButton::clicked, this, [this, page = quickActions.at(index).second] {
            emit navigateRequested(page);
        });
    }
    layout->addLayout(quickLayout);
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
