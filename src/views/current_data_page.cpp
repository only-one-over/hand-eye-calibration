#include "views/current_data_page.h"

#include "models/calibration_session_model.h"

#include <QAbstractItemView>
#include <QHBoxLayout>
#include <QLabel>
#include <QFont>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QSplitter>
#include <QTableView>
#include <QHeaderView>
#include <QVBoxLayout>

namespace handeye {

CurrentDataPage::CurrentDataPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(18, 18, 18, 18);

    auto *title = new QLabel(QStringLiteral("当前数据"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *splitter = new QSplitter(Qt::Horizontal, this);
    m_model = new SampleTableModel(this);
    m_table = new QTableView(splitter);
    m_table->setObjectName(QStringLiteral("currentDataTable"));
    m_table->setModel(m_model);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    auto *previewPanel = new QWidget(splitter);
    auto *previewLayout = new QVBoxLayout(previewPanel);
    previewLayout->addWidget(new QLabel(QStringLiteral("选中样本图片"), previewPanel));
    m_imagePreview = new QLabel(QStringLiteral("请选择一个样本"), previewPanel);
    m_imagePreview->setAlignment(Qt::AlignCenter);
    m_imagePreview->setMinimumSize(300, 240);
    m_imagePreview->setStyleSheet(QStringLiteral("border: 1px solid #E0E0E0; background: #FAFAFA;"));
    previewLayout->addWidget(m_imagePreview, 1);
    m_imageInfo = new QLabel(QStringLiteral("图片路径：\n状态："), previewPanel);
    m_imageInfo->setWordWrap(true);
    previewLayout->addWidget(m_imageInfo);
    auto *deleteButton = new QPushButton(QStringLiteral("删除选中样本"), previewPanel);
    previewLayout->addWidget(deleteButton);
    connect(deleteButton, &QPushButton::clicked, this, &CurrentDataPage::deleteSelected);

    splitter->addWidget(m_table);
    splitter->addWidget(previewPanel);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);
    layout->addWidget(splitter, 1);

    connect(m_table->selectionModel(), &QItemSelectionModel::currentRowChanged,
            this, &CurrentDataPage::onCurrentRowChanged);
}

void CurrentDataPage::setSamples(const QVector<PoseSample> &samples)
{
    m_samples = samples;
    m_model->setSamples(samples);
    if (samples.isEmpty()) {
        m_currentPixmap = {};
        m_imagePreview->setText(QStringLiteral("请选择一个样本"));
        m_imagePreview->setPixmap({});
        m_imageInfo->setText(QStringLiteral("图片路径：\n状态："));
        return;
    }
    m_table->selectRow(0);
    onCurrentRowChanged(m_model->index(0, 0), {});
}

void CurrentDataPage::onCurrentRowChanged(const QModelIndex &current, const QModelIndex &)
{
    if (!current.isValid() || current.row() < 0 || current.row() >= m_samples.size()) return;
    const PoseSample &sample = m_samples.at(current.row());
    m_currentPixmap = sample.imagePath.isEmpty() ? QPixmap{} : QPixmap(sample.imagePath);
    const QString previewText = sample.imageStatus == ImageSampleStatus::ManualPose
                                    ? QStringLiteral("手动输入位姿，无图片预览")
                                    : QStringLiteral("请选择一个样本");
    if (m_currentPixmap.isNull()) m_imagePreview->setText(previewText);
    m_imageInfo->setText(QStringLiteral("图片路径：%1\n状态：%2\n角点数：%3\nPnP RMSE：%4 px\n说明：%5")
                             .arg(sample.imagePath, imageSampleStatusName(sample.imageStatus))
                             .arg(sample.detectedCornerCount)
                             .arg(sample.pnpReprojectionRmsePx, 0, 'f', 3)
                             .arg(sample.imageMessage));
    updatePreview();
}

void CurrentDataPage::deleteSelected()
{
    if (!m_table->selectionModel()) return;
    const QVector<int> ids = m_model->idsAt(m_table->selectionModel()->selectedRows());
    if (!ids.isEmpty()) emit deleteRequested(ids);
}

void CurrentDataPage::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    updatePreview();
}

void CurrentDataPage::updatePreview()
{
    if (m_currentPixmap.isNull()) {
        m_imagePreview->setPixmap({});
        if (m_samples.isEmpty()) m_imagePreview->setText(QStringLiteral("请选择一个样本"));
        return;
    }
    m_imagePreview->setText({});
    m_imagePreview->setPixmap(m_currentPixmap.scaled(m_imagePreview->size(), Qt::KeepAspectRatio,
                                                     Qt::SmoothTransformation));
}

} // namespace handeye
