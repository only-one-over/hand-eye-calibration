#include "core/board_pdf_storage.h"

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <QUrl>

namespace handeye {

namespace {

QString numberToken(double value, int decimals)
{
    return QString::number(value, 'f', decimals).replace(QLatin1Char('.'), QLatin1Char('p'));
}

QString patternToken(BoardPattern pattern)
{
    switch (pattern) {
    case BoardPattern::Chessboard: return QStringLiteral("chessboard");
    case BoardPattern::Charuco: return QStringLiteral("charuco");
    case BoardPattern::ArucoGrid: return QStringLiteral("aruco_grid");
    }
    return QStringLiteral("board");
}

QString modeToken(BoardPdfOutputMode mode)
{
    return mode == BoardPdfOutputMode::A4Tiled ? QStringLiteral("a4") : QStringLiteral("single");
}

QString baseFileName(const BoardSpec &board, BoardPdfOutputMode mode)
{
    QStringList parts;
    parts.append(patternToken(board.pattern));

    if (board.pattern == BoardPattern::Chessboard || board.pattern == BoardPattern::Charuco) {
        parts.append(QStringLiteral("%1x%2").arg(board.innerCornersX).arg(board.innerCornersY));
        parts.append(QStringLiteral("%1mm").arg(numberToken(board.squareSizeM * 1000.0, 3)));
        if (board.pattern == BoardPattern::Charuco) {
            parts.append(QStringLiteral("marker%1mm")
                             .arg(numberToken(board.markerSizeM * 1000.0, 3)));
            parts.append(QStringLiteral("dict%1").arg(board.arucoDictionary));
        }
    } else {
        parts.append(QStringLiteral("%1x%2").arg(board.markerCountX).arg(board.markerCountY));
        parts.append(QStringLiteral("marker%1mm")
                         .arg(numberToken(board.markerSizeM * 1000.0, 3)));
        parts.append(QStringLiteral("gap%1mm")
                         .arg(numberToken(board.markerSeparationM * 1000.0, 3)));
        parts.append(QStringLiteral("dict%1").arg(board.arucoDictionary));
    }

    parts.append(modeToken(mode));
    return parts.join(QLatin1Char('_'));
}

QString writableRoot()
{
    QString root = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (root.isEmpty()) root = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    return root;
}

} // namespace

QString BoardPdfStorage::directory()
{
    const QString root = writableRoot();
    if (root.isEmpty()) return {};
    return QDir(root).filePath(QStringLiteral("HandEyeCalibration/board_pdfs"));
}

bool BoardPdfStorage::ensureDirectory(QString *error)
{
    const QString path = directory();
    if (path.isEmpty()) {
        if (error) *error = QStringLiteral("无法确定可写的用户文档目录。\n");
        return false;
    }
    if (QDir().mkpath(path)) return true;
    if (error) *error = QStringLiteral("无法创建 PDF 输出目录：") + path;
    return false;
}

QString BoardPdfStorage::suggestedFileName(const BoardSpec &board, BoardPdfOutputMode mode)
{
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    return baseFileName(board, mode) + QLatin1Char('_') + timestamp + QStringLiteral(".pdf");
}

QString BoardPdfStorage::findExistingPath(const BoardSpec &board, BoardPdfOutputMode mode)
{
    const QString outputDirectory = directory();
    if (outputDirectory.isEmpty() || !QFileInfo(outputDirectory).isDir()) return {};

    const QString prefix = baseFileName(board, mode) + QLatin1Char('_');
    const QFileInfoList files = QDir(outputDirectory).entryInfoList(
        QDir::Files | QDir::Readable, QDir::Time | QDir::Reversed);
    QFileInfo newest;
    for (const QFileInfo &file : files) {
        if (!file.fileName().startsWith(prefix, Qt::CaseSensitive)
            || !file.fileName().endsWith(QStringLiteral(".pdf"), Qt::CaseInsensitive)
            || file.size() <= 100) {
            continue;
        }
        if (!newest.exists() || file.lastModified() > newest.lastModified()
            || (file.lastModified() == newest.lastModified() && file.fileName() > newest.fileName())) {
            newest = file;
        }
    }
    return newest.exists() ? newest.absoluteFilePath() : QString();
}

QString BoardPdfStorage::nextOutputPath(const BoardSpec &board, BoardPdfOutputMode mode,
                                        QString *error)
{
    if (!ensureDirectory(error)) return {};

    const QString outputDirectory = directory();
    const QString fileName = suggestedFileName(board, mode);
    const QFileInfo initialInfo(QDir(outputDirectory).filePath(fileName));
    QString candidate = initialInfo.absoluteFilePath();
    if (!QFileInfo::exists(candidate)) return candidate;

    const QString stem = initialInfo.completeBaseName();
    const QString suffix = initialInfo.completeSuffix();
    for (int index = 2; index < 1000000; ++index) {
        const QString numberedName = QStringLiteral("%1_%2.%3").arg(stem).arg(index).arg(suffix);
        candidate = QDir(outputDirectory).filePath(numberedName);
        if (!QFileInfo::exists(candidate)) return QFileInfo(candidate).absoluteFilePath();
    }

    if (error) *error = QStringLiteral("无法为 PDF 找到可用的文件名。\n");
    return {};
}

bool BoardPdfStorage::openDirectory(QString *error)
{
    if (!ensureDirectory(error)) return false;
    const QString path = directory();
    if (QDesktopServices::openUrl(QUrl::fromLocalFile(path))) return true;
    if (error) *error = QStringLiteral("系统无法打开 PDF 输出目录：") + path;
    return false;
}

} // namespace handeye
