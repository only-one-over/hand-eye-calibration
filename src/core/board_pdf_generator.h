#pragma once

#include "domain/calibration_types.h"

namespace handeye {

enum class BoardPdfOutputMode { CustomSize, A4Tiled };

QString boardPdfOutputModeName(BoardPdfOutputMode mode);

class BoardPdfGenerator
{
public:
    static BoardPdfReport generate(const BoardSpec &board,
                                   const QString &outputPath,
                                   BoardPdfOutputMode mode);
    static BoardPdfReport describeExisting(const BoardSpec &board,
                                           const QString &outputPath,
                                           BoardPdfOutputMode mode);
};

} // namespace handeye
