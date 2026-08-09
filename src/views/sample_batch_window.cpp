#include "views/sample_batch_window.h"

#include "models/calibration_session_model.h"

#include <QAbstractItemView>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>

namespace handeye {

SampleBatchWindow::SampleBatchWindow(QWidget *parent) : QDialog(parent)
{
    setWindowTitle(QStringLiteral("当前轮次 - 机器人坐标与标定板位姿"));
    resize(1500, 760);
    setModal(false);

    auto *layout = new QVBoxLayout(this);
    auto *hint = new QLabel(
        QStringLiteral("每一行表示同一轮采集中的一组对应样本：机器人 gripper→base、标定板图片和计算出的 target→camera。"),
        this);
    hint->setWordWrap(true);
    layout->addWidget(hint);

    m_summary = new QLabel(QStringLiteral("当前没有样本。"), this);
    layout->addWidget(m_summary);

    m_model = new SampleTableModel(this);
    m_table = new QTableView(this);
    m_table->setModel(m_model);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    layout->addWidget(m_table, 1);

    m_deleteButton = new QPushButton(QStringLiteral("删除选中样本"), this);
    layout->addWidget(m_deleteButton);
    connect(m_deleteButton, &QPushButton::clicked, this, &SampleBatchWindow::deleteSelected);
}

void SampleBatchWindow::setSamples(const QVector<PoseSample> &samples)
{
    m_model->setSamples(samples);
    int images = 0;
    int poses = 0;
    for (const PoseSample &sample : samples) {
        if (!sample.imagePath.isEmpty()) ++images;
        if (sample.imageStatus == ImageSampleStatus::PoseEstimated) ++poses;
    }
    m_summary->setText(QStringLiteral("本轮样本：%1 组 | 图片：%2 张 | 已生成 target→camera：%3 组")
                          .arg(samples.size()).arg(images).arg(poses));
}

void SampleBatchWindow::deleteSelected()
{
    if (!m_table || !m_table->selectionModel()) return;
    const QVector<int> ids = m_model->idsAt(m_table->selectionModel()->selectedRows());
    if (!ids.isEmpty()) emit deleteRequested(ids);
}

} // namespace handeye
