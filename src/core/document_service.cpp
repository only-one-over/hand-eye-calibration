#include "core/document_service.h"

#include <QCoreApplication>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QUrl>

namespace handeye {

namespace {

struct DocumentDefinition {
    const char *fileName;
    const char *title;
};

constexpr DocumentDefinition kDocuments[] = {
    {"hand_eye_quick_start.pdf", "手眼标定快速使用说明"},
    {"checkerboard_printing.pdf", "标定板打印与尺寸检查"},
    {"data_collection_quality.pdf", "标定数据采集质量建议"},
};

QString resourcePath(const QString &fileName)
{
    return QStringLiteral(":/docs/") + fileName;
}

QString materializedResourcePath(const QString &fileName)
{
    const QString directory = QStandardPaths::writableLocation(QStandardPaths::TempLocation)
                               + QStringLiteral("/hand_eye_calibration_docs");
    QDir().mkpath(directory);
    return QDir(directory).filePath(fileName);
}

} // namespace

QString DocumentService::documentsDirectory()
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("docs"));
}

DocumentInfo DocumentService::resolve(const QString &fileName)
{
    DocumentInfo info;
    info.fileName = fileName;
    for (const DocumentDefinition &definition : kDocuments) {
        if (fileName == QLatin1String(definition.fileName)) {
            info.title = QString::fromUtf8(definition.title);
            break;
        }
    }
    const QString externalPath = QDir(documentsDirectory()).filePath(fileName);
    if (QFileInfo(externalPath).isFile()) {
        info.available = true;
        info.external = true;
        info.source = QStringLiteral("外置 docs/ 文件");
        info.resolvedPath = externalPath;
        return info;
    }
    if (QFile::exists(resourcePath(fileName))) {
        info.available = true;
        info.source = QStringLiteral("内置资源");
        info.resolvedPath = materializedResourcePath(fileName);
    }
    return info;
}

QVector<DocumentInfo> DocumentService::listDocuments()
{
    QVector<DocumentInfo> result;
    for (const DocumentDefinition &definition : kDocuments)
        result.append(resolve(QString::fromUtf8(definition.fileName)));
    return result;
}

bool DocumentService::openDocument(const QString &fileName, QString *error)
{
    const DocumentInfo info = resolve(fileName);
    if (!info.available) {
        if (error) *error = QStringLiteral("说明文档不存在：") + fileName;
        return false;
    }
    QString path = info.resolvedPath;
    if (!info.external) {
        QFile resource(resourcePath(fileName));
        if (!resource.open(QIODevice::ReadOnly)) {
            if (error) *error = QStringLiteral("无法读取内置说明文档：") + fileName;
            return false;
        }
        QFile output(path);
        if (!output.open(QIODevice::WriteOnly)) {
            if (error) *error = QStringLiteral("无法释放内置说明文档：") + output.errorString();
            return false;
        }
        output.write(resource.readAll());
        output.close();
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(path))) {
        if (error) *error = QStringLiteral("系统无法打开说明文档：") + path;
        return false;
    }
    return true;
}

bool DocumentService::openDocumentsDirectory(QString *error)
{
    const QString directory = documentsDirectory();
    if (!QDir().mkpath(directory)) {
        if (error) *error = QStringLiteral("无法创建文档目录：") + directory;
        return false;
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(directory))) {
        if (error) *error = QStringLiteral("系统无法打开文档目录：") + directory;
        return false;
    }
    return true;
}

} // namespace handeye
