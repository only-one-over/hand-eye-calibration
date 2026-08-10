#include "core/board_pdf_generator.h"

#include <QDateTime>
#include <QFileInfo>
#include <QImage>
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QPdfWriter>

#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <algorithm>
#include <cmath>

namespace handeye {

namespace {

constexpr double kMmPerM = 1000.0;
constexpr double kA4WidthMm = 210.0;
constexpr double kA4HeightMm = 297.0;
constexpr double kPageMarginMm = 10.0;
constexpr double kTileOverlapMm = 5.0;
constexpr int kRasterPixelsPerMm = 12;

struct BoardGeometry {
    double widthMm = 0.0;
    double heightMm = 0.0;
};

bool finitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

bool validBoard(const BoardSpec &board, QString *error)
{
    switch (board.pattern) {
    case BoardPattern::Chessboard:
        if (board.innerCornersX < 2 || board.innerCornersY < 2 || !finitePositive(board.squareSizeM)) {
            if (error) *error = QStringLiteral("Chessboard 行列数和方格尺寸必须为正。\n");
            return false;
        }
        break;
    case BoardPattern::Charuco:
        if (board.innerCornersX < 1 || board.innerCornersY < 1 || !finitePositive(board.squareSizeM)
            || !finitePositive(board.markerSizeM) || board.markerSizeM >= board.squareSizeM) {
            if (error) *error = QStringLiteral("ChArUco 需要有效的行列数、方格尺寸和 marker 尺寸，且 marker 必须小于方格。\n");
            return false;
        }
        break;
    case BoardPattern::ArucoGrid:
        if (board.markerCountX < 1 || board.markerCountY < 1 || !finitePositive(board.markerSizeM)
            || !std::isfinite(board.markerSeparationM) || board.markerSeparationM < 0.0) {
            if (error) *error = QStringLiteral("ArUco Grid 需要有效的 marker 行列、尺寸和间距。\n");
            return false;
        }
        break;
    }
    return true;
}

BoardGeometry boardGeometry(const BoardSpec &board)
{
    if (board.pattern == BoardPattern::Chessboard || board.pattern == BoardPattern::Charuco) {
        return {(board.innerCornersX + 1) * board.squareSizeM * kMmPerM,
                (board.innerCornersY + 1) * board.squareSizeM * kMmPerM};
    }
    return {board.markerCountX * board.markerSizeM * kMmPerM
                + std::max(0, board.markerCountX - 1) * board.markerSeparationM * kMmPerM,
            board.markerCountY * board.markerSizeM * kMmPerM
                + std::max(0, board.markerCountY - 1) * board.markerSeparationM * kMmPerM};
}

void a4TileCounts(const BoardGeometry &geometry, int *columns, int *rows)
{
    const bool landscape = geometry.widthMm / std::max(geometry.heightMm, 0.001)
                           > kA4WidthMm / kA4HeightMm;
    const double pageWidthMm = landscape ? kA4HeightMm : kA4WidthMm;
    const double pageHeightMm = landscape ? kA4WidthMm : kA4HeightMm;
    const double contentWidthMm = pageWidthMm - 2.0 * kPageMarginMm;
    const double contentHeightMm = pageHeightMm - 2.0 * kPageMarginMm;
    const double stepX = std::max(1.0, contentWidthMm - kTileOverlapMm);
    const double stepY = std::max(1.0, contentHeightMm - kTileOverlapMm);
    if (columns)
        *columns = std::max(1, static_cast<int>(std::ceil((geometry.widthMm - kTileOverlapMm) / stepX)));
    if (rows)
        *rows = std::max(1, static_cast<int>(std::ceil((geometry.heightMm - kTileOverlapMm) / stepY)));
}

QImage matToImage(const cv::Mat &mat)
{
    if (mat.empty()) return {};
    if (mat.type() == CV_8UC1)
        return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8).copy();
    if (mat.type() == CV_8UC3) {
        QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_BGR888);
        return image.copy();
    }
    return {};
}

QImage makeBoardImage(const BoardSpec &board, const BoardGeometry &geometry, QString *error)
{
    const int widthPx = std::max(2, static_cast<int>(std::lround(geometry.widthMm * kRasterPixelsPerMm)));
    const int heightPx = std::max(2, static_cast<int>(std::lround(geometry.heightMm * kRasterPixelsPerMm)));

    if (board.pattern == BoardPattern::Chessboard) {
        QImage image(widthPx, heightPx, QImage::Format_RGB32);
        image.fill(Qt::white);
        QPainter painter(&image);
        const double squareWidth = static_cast<double>(widthPx) / (board.innerCornersX + 1);
        const double squareHeight = static_cast<double>(heightPx) / (board.innerCornersY + 1);
        painter.setPen(Qt::NoPen);
        painter.setBrush(Qt::black);
        for (int row = 0; row < board.innerCornersY + 1; ++row) {
            for (int col = 0; col < board.innerCornersX + 1; ++col) {
                if ((row + col) % 2 == 0)
                    painter.drawRect(QRectF(col * squareWidth, row * squareHeight,
                                            squareWidth + 0.5, squareHeight + 0.5));
            }
        }
        return image;
    }

    try {
        const cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(board.arucoDictionary);
        cv::Mat generated;
        if (board.pattern == BoardPattern::Charuco) {
            const cv::aruco::CharucoBoard charuco(
                cv::Size(board.innerCornersX + 1, board.innerCornersY + 1),
                static_cast<float>(board.squareSizeM), static_cast<float>(board.markerSizeM), dictionary);
            charuco.generateImage(cv::Size(widthPx, heightPx), generated, 0, 1);
        } else {
            const cv::aruco::GridBoard grid(
                cv::Size(board.markerCountX, board.markerCountY), static_cast<float>(board.markerSizeM),
                static_cast<float>(board.markerSeparationM), dictionary);
            grid.generateImage(cv::Size(widthPx, heightPx), generated, 0, 1);
        }
        QImage image = matToImage(generated);
        if (!image.isNull()) return image;
    } catch (const cv::Exception &exception) {
        if (error) *error = QString::fromUtf8(exception.what());
        return {};
    }
    if (error) *error = QStringLiteral("OpenCV 无法生成当前标定板图案。\n");
    return {};
}

void drawRuler(QPainter &painter, double x, double y)
{
    painter.setPen(QPen(Qt::black, 0.25));
    painter.drawLine(QPointF(x, y), QPointF(x + 100.0, y));
    for (int millimeter = 0; millimeter <= 100; millimeter += 10) {
        const double tickHeight = millimeter % 50 == 0 ? 4.0 : 2.5;
        painter.drawLine(QPointF(x + millimeter, y), QPointF(x + millimeter, y - tickHeight));
    }
}

void drawBoard(QPainter &painter, const QImage &image, const BoardGeometry &geometry,
               double x, double y)
{
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter.drawImage(QRectF(x, y, geometry.widthMm, geometry.heightMm), image);
}

void drawPageFrame(QPainter &painter, double pageWidthMm, double pageHeightMm)
{
    painter.setPen(QPen(Qt::darkGray, 0.25));
    painter.drawRect(QRectF(kPageMarginMm, kPageMarginMm,
                            pageWidthMm - 2.0 * kPageMarginMm,
                            pageHeightMm - 2.0 * kPageMarginMm));
}

} // namespace

QString boardPdfOutputModeName(BoardPdfOutputMode mode)
{
    return mode == BoardPdfOutputMode::A4Tiled ? QStringLiteral("A4 1:1 分块")
                                               : QStringLiteral("自定义物理尺寸");
}

BoardPdfReport BoardPdfGenerator::generate(const BoardSpec &board, const QString &outputPath,
                                           BoardPdfOutputMode mode)
{
    BoardPdfReport report;
    report.pattern = boardPatternName(board.pattern);
    report.outputMode = boardPdfOutputModeName(mode);
    report.generatedAt = QDateTime::currentDateTime();
    if (outputPath.trimmed().isEmpty()) {
        report.error = QStringLiteral("输出路径为空。\n");
        return report;
    }
    if (QFileInfo::exists(outputPath)) {
        report.error = QStringLiteral("目标文件已存在，为避免覆盖请另选文件名。\n");
        return report;
    }
    if (!validBoard(board, &report.error)) return report;

    const BoardGeometry geometry = boardGeometry(board);
    if (geometry.widthMm > 2000.0 || geometry.heightMm > 2000.0) {
        report.error = QStringLiteral("标定板尺寸超过 2000 mm，已拒绝生成。\n");
        return report;
    }
    report.widthMm = geometry.widthMm;
    report.heightMm = geometry.heightMm;

    QString imageError;
    const QImage boardImage = makeBoardImage(board, geometry, &imageError);
    if (boardImage.isNull()) {
        report.error = QStringLiteral("生成标定板图案失败：") + imageError;
        return report;
    }

    QPdfWriter writer(outputPath);
    writer.setResolution(300);
    writer.setTitle(QStringLiteral("Hand-Eye Calibration Board"));
    writer.setCreator(QStringLiteral("Qt6 Hand-Eye Calibration"));
    writer.setPageMargins(QMarginsF(0, 0, 0, 0));

    const double customPageWidthMm = geometry.widthMm;
    const double customPageHeightMm = geometry.heightMm;
    const bool a4Landscape = geometry.widthMm / std::max(geometry.heightMm, 0.001)
                             > kA4WidthMm / kA4HeightMm;
    const double initialPageWidthMm = mode == BoardPdfOutputMode::CustomSize
                                          ? customPageWidthMm
                                          : (a4Landscape ? kA4HeightMm : kA4WidthMm);
    const double initialPageHeightMm = mode == BoardPdfOutputMode::CustomSize
                                           ? customPageHeightMm
                                           : (a4Landscape ? kA4WidthMm : kA4HeightMm);
    writer.setPageSize(QPageSize(QSizeF(initialPageWidthMm, initialPageHeightMm), QPageSize::Millimeter));

    QPainter painter;
    if (!painter.begin(&writer)) {
        report.error = QStringLiteral("无法打开 PDF 输出文件。\n");
        return report;
    }

    auto setupPage = [&writer, &painter](double pageWidthMm, double pageHeightMm) {
        const double scale = static_cast<double>(writer.width()) / pageWidthMm;
        painter.resetTransform();
        painter.scale(scale, scale);
    };

    if (mode == BoardPdfOutputMode::CustomSize) {
        const double pageWidthMm = customPageWidthMm;
        const double pageHeightMm = customPageHeightMm;
        setupPage(pageWidthMm, pageHeightMm);
        drawBoard(painter, boardImage, geometry, 0.0, 0.0);
        report.pageCount = 1;
    } else {
        const bool landscape = a4Landscape;
        const double pageWidthMm = landscape ? kA4HeightMm : kA4WidthMm;
        const double pageHeightMm = landscape ? kA4WidthMm : kA4HeightMm;
        const double contentWidthMm = pageWidthMm - 2.0 * kPageMarginMm;
        const double contentHeightMm = pageHeightMm - 2.0 * kPageMarginMm;
        const double stepX = std::max(1.0, contentWidthMm - kTileOverlapMm);
        const double stepY = std::max(1.0, contentHeightMm - kTileOverlapMm);
        int columns = 1;
        int rows = 1;
        a4TileCounts(geometry, &columns, &rows);
        report.pageCount = columns * rows;
        if (report.pageCount > 1)
            report.warnings.append(QStringLiteral("A4 版本需要拼接，请保持 100%% 实际尺寸打印。"));

        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < columns; ++col) {
                if (row != 0 || col != 0) writer.newPage();
                setupPage(pageWidthMm, pageHeightMm);
                painter.save();
                painter.setClipRect(QRectF(kPageMarginMm, kPageMarginMm, contentWidthMm, contentHeightMm));
                painter.translate(kPageMarginMm - col * stepX, kPageMarginMm - row * stepY);
                drawBoard(painter, boardImage, geometry, 0.0, 0.0);
                painter.restore();
                drawPageFrame(painter, pageWidthMm, pageHeightMm);
                drawRuler(painter, pageWidthMm - 110.0, pageHeightMm - 18.0);
            }
        }
    }

    painter.end();
    if (!QFileInfo::exists(outputPath)) {
        report.error = QStringLiteral("PDF 生成结束但文件不存在。\n");
        return report;
    }
    report.outputPath = outputPath;
    report.success = true;
    return report;
}

BoardPdfReport BoardPdfGenerator::describeExisting(const BoardSpec &board,
                                                   const QString &outputPath,
                                                   BoardPdfOutputMode mode)
{
    BoardPdfReport report;
    report.pattern = boardPatternName(board.pattern);
    report.outputMode = boardPdfOutputModeName(mode);
    const QFileInfo fileInfo(outputPath);
    if (!fileInfo.isFile() || fileInfo.size() <= 100) {
        report.error = QStringLiteral("已有 PDF 文件不存在或文件内容不完整。\n");
        return report;
    }
    if (!validBoard(board, &report.error)) return report;

    const BoardGeometry geometry = boardGeometry(board);
    report.outputPath = fileInfo.absoluteFilePath();
    report.widthMm = geometry.widthMm;
    report.heightMm = geometry.heightMm;
    report.generatedAt = fileInfo.lastModified();
    if (mode == BoardPdfOutputMode::CustomSize) {
        report.pageCount = 1;
    } else {
        int columns = 1;
        int rows = 1;
        a4TileCounts(geometry, &columns, &rows);
        report.pageCount = columns * rows;
        if (report.pageCount > 1)
            report.warnings.append(QStringLiteral("A4 版本需要拼接，请保持 100%% 实际尺寸打印。"));
    }
    report.success = true;
    report.reused = true;
    return report;
}

} // namespace handeye
