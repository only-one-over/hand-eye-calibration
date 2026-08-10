#pragma once

#include "core/board_pdf_generator.h"

#include <QString>

namespace handeye {

class BoardPdfStorage
{
public:
    static QString directory();
    static bool ensureDirectory(QString *error = nullptr);
    static QString suggestedFileName(const BoardSpec &board, BoardPdfOutputMode mode);
    static QString findExistingPath(const BoardSpec &board, BoardPdfOutputMode mode);
    static QString nextOutputPath(const BoardSpec &board, BoardPdfOutputMode mode,
                                  QString *error = nullptr);
    static bool openDirectory(QString *error = nullptr);
};

} // namespace handeye
