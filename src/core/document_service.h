#pragma once

#include <QString>
#include <QVector>

namespace handeye {

struct DocumentInfo {
    QString fileName;
    QString title;
    QString resolvedPath;
    QString source;
    bool available = false;
    bool external = false;
};

class DocumentService
{
public:
    static QString documentsDirectory();
    static QVector<DocumentInfo> listDocuments();
    static DocumentInfo resolve(const QString &fileName);
    static bool openDocument(const QString &fileName, QString *error = nullptr);
    static bool openDocumentsDirectory(QString *error = nullptr);
};

} // namespace handeye
