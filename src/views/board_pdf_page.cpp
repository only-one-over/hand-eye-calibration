#include "views/board_pdf_page.h"

#include "core/board_pdf_storage.h"
#include "core/document_service.h"

#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>

namespace handeye {

namespace {

void boardDimensionsMm(const BoardSpec &board, double *widthMm, double *heightMm)
{
    if (board.pattern == BoardPattern::Chessboard || board.pattern == BoardPattern::Charuco) {
        *widthMm = (board.innerCornersX + 1) * board.squareSizeM * 1000.0;
        *heightMm = (board.innerCornersY + 1) * board.squareSizeM * 1000.0;
    } else {
        *widthMm = (board.markerCountX * board.markerSizeM
                    + std::max(0, board.markerCountX - 1) * board.markerSeparationM) * 1000.0;
        *heightMm = (board.markerCountY * board.markerSizeM
                     + std::max(0, board.markerCountY - 1) * board.markerSeparationM) * 1000.0;
    }
}

} // namespace

BoardPdfPage::BoardPdfPage(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(24, 24, 24, 24);
    layout->setSpacing(16);

    auto *title = new QLabel(QStringLiteral("标定板 PDF"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    title->setFont(titleFont);
    layout->addWidget(title);

    auto *intro = new QLabel(QStringLiteral(
        "根据参数页当前 BoardSpec 生成可打印标定板。PDF 图案区域不再附带大段说明文字；打印时请选择 100% / 实际大小，关闭适应页面。"), this);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto *summaryGroup = new QGroupBox(QStringLiteral("当前标定板参数"), this);
    auto *summaryLayout = new QVBoxLayout(summaryGroup);
    m_boardSummary = new QLabel(summaryGroup);
    m_boardSummary->setWordWrap(true);
    summaryLayout->addWidget(m_boardSummary);
    layout->addWidget(summaryGroup);

    auto *generateGroup = new QGroupBox(QStringLiteral("生成打印文件"), this);
    auto *generateLayout = new QVBoxLayout(generateGroup);
    const QString outputDirectory = BoardPdfStorage::directory();
    auto *directoryLabel = new QLabel(
        outputDirectory.isEmpty()
            ? QStringLiteral("默认输出目录：无法确定用户文档目录")
            : QStringLiteral("默认输出目录：%1").arg(outputDirectory),
        generateGroup);
    directoryLabel->setObjectName(QStringLiteral("boardPdfOutputDirectoryLabel"));
    directoryLabel->setWordWrap(true);
    generateLayout->addWidget(directoryLabel);
    auto *directLayout = new QHBoxLayout;
    auto *customButton = new QPushButton(QStringLiteral("生成 1:1 单页 PDF"), generateGroup);
    customButton->setObjectName(QStringLiteral("generateCustomBoardPdfButton"));
    customButton->setProperty("variant", "primary");
    auto *a4Button = new QPushButton(QStringLiteral("生成 A4 分块 PDF"), generateGroup);
    a4Button->setObjectName(QStringLiteral("generateA4BoardPdfButton"));
    directLayout->addWidget(customButton);
    directLayout->addWidget(a4Button);
    generateLayout->addLayout(directLayout);
    auto *saveAsLayout = new QHBoxLayout;
    auto *saveAsCustomButton = new QPushButton(QStringLiteral("另存 1:1 单页 PDF"), generateGroup);
    saveAsCustomButton->setObjectName(QStringLiteral("saveAsCustomBoardPdfButton"));
    auto *saveAsA4Button = new QPushButton(QStringLiteral("另存 A4 分块 PDF"), generateGroup);
    saveAsA4Button->setObjectName(QStringLiteral("saveAsA4BoardPdfButton"));
    saveAsLayout->addWidget(saveAsCustomButton);
    saveAsLayout->addWidget(saveAsA4Button);
    generateLayout->addLayout(saveAsLayout);
    layout->addWidget(generateGroup);
    connect(customButton, &QPushButton::clicked, this,
            [this] { emit generateRequested(BoardPdfOutputMode::CustomSize); });
    connect(a4Button, &QPushButton::clicked, this,
            [this] { emit generateRequested(BoardPdfOutputMode::A4Tiled); });
    connect(saveAsCustomButton, &QPushButton::clicked, this,
            [this] { emit saveAsRequested(BoardPdfOutputMode::CustomSize); });
    connect(saveAsA4Button, &QPushButton::clicked, this,
            [this] { emit saveAsRequested(BoardPdfOutputMode::A4Tiled); });

    auto *outputGroup = new QGroupBox(QStringLiteral("最近生成结果"), this);
    auto *outputLayout = new QVBoxLayout(outputGroup);
    m_outputStatus = new QLabel(QStringLiteral("尚未生成标定板 PDF。"), outputGroup);
    m_outputStatus->setWordWrap(true);
    outputLayout->addWidget(m_outputStatus);
    auto *outputButtons = new QHBoxLayout;
    m_openPdfButton = new QPushButton(QStringLiteral("打开最近生成的 PDF"), outputGroup);
    m_openPdfButton->setObjectName(QStringLiteral("openGeneratedBoardPdfButton"));
    m_openPdfButton->setEnabled(false);
    m_openDirectoryButton = new QPushButton(QStringLiteral("打开输出目录"), outputGroup);
    m_openDirectoryButton->setObjectName(QStringLiteral("openBoardPdfDirectoryButton"));
    outputButtons->addWidget(m_openPdfButton);
    outputButtons->addWidget(m_openDirectoryButton);
    outputLayout->addLayout(outputButtons);
    layout->addWidget(outputGroup);
    connect(m_openPdfButton, &QPushButton::clicked, this,
            [this] { emit openBoardPdfRequested(m_lastPdfPath); });
    connect(m_openDirectoryButton, &QPushButton::clicked, this,
            [this] { emit openBoardPdfDirectoryRequested(); });

    auto *docsGroup = new QGroupBox(QStringLiteral("固定说明文档"), this);
    auto *docsLayout = new QVBoxLayout(docsGroup);
    const QVector<DocumentInfo> documents = DocumentService::listDocuments();
    for (const DocumentInfo &document : documents) {
        auto *button = new QPushButton(
            QStringLiteral("%1（%2）").arg(document.title, document.source), docsGroup);
        button->setObjectName(QStringLiteral("openDocument_%1").arg(document.fileName));
        button->setEnabled(document.available);
        connect(button, &QPushButton::clicked, this,
                [this, fileName = document.fileName] { emit openDocumentRequested(fileName); });
        docsLayout->addWidget(button);
    }
    auto *openDocsDirectoryButton = new QPushButton(QStringLiteral("打开说明文档目录"), docsGroup);
    openDocsDirectoryButton->setObjectName(QStringLiteral("openDocumentsDirectoryButton"));
    docsLayout->addWidget(openDocsDirectoryButton);
    connect(openDocsDirectoryButton, &QPushButton::clicked, this,
            &BoardPdfPage::openDocumentsDirectoryRequested);
    layout->addWidget(docsGroup);
    layout->addStretch();

    updateBoardSummary();
}

void BoardPdfPage::setBoardSpec(const BoardSpec &board)
{
    m_boardSpec = board;
    updateBoardSummary();
}

void BoardPdfPage::updateBoardSummary()
{
    double widthMm = 0.0;
    double heightMm = 0.0;
    boardDimensionsMm(m_boardSpec, &widthMm, &heightMm);
    m_boardSummary->setText(QStringLiteral(
        "板型：%1\n有效物理尺寸：%2 × %3 mm\n"
        "Chessboard 内角点：%4 × %5 | 方格：%6 mm\n"
        "ArUco 字典：%7 | marker：%8 × %9 mm | 间距：%10 mm")
                                .arg(boardPatternName(m_boardSpec.pattern))
                                .arg(widthMm, 0, 'f', 2)
                                .arg(heightMm, 0, 'f', 2)
                                .arg(m_boardSpec.innerCornersX)
                                .arg(m_boardSpec.innerCornersY)
                                .arg(m_boardSpec.squareSizeM * 1000.0, 0, 'f', 3)
                                .arg(m_boardSpec.arucoDictionary)
                                .arg(m_boardSpec.markerSizeM * 1000.0, 0, 'f', 3)
                                .arg(m_boardSpec.markerSeparationM * 1000.0, 0, 'f', 3));
}

void BoardPdfPage::setReport(const BoardPdfReport &report)
{
    if (!report.success) {
        m_outputStatus->setText(report.error.isEmpty()
                                    ? QStringLiteral("尚未生成标定板 PDF。")
                                    : QStringLiteral("生成失败：") + report.error);
        m_openPdfButton->setEnabled(false);
        return;
    }
    m_lastPdfPath = report.outputPath;
    const QString reuseStatus = report.reused ? QStringLiteral("\n状态：已复用同规格 PDF") : QString();
    m_outputStatus->setText(QStringLiteral("文件：%1\n模式：%2 | 页数：%3 | 标定板尺寸：%4 × %5 mm%6")
                                .arg(report.outputPath)
                                .arg(report.outputMode)
                                .arg(report.pageCount)
                                .arg(report.widthMm, 0, 'f', 2)
                                .arg(report.heightMm, 0, 'f', 2)
                                .arg(reuseStatus));
    m_openPdfButton->setEnabled(true);
    m_openDirectoryButton->setEnabled(true);
}

} // namespace handeye
