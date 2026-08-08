#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct AdapterResult {
    bool success = false;
    Vector3 rotation{};
    Vector3 translation{};
    QString error;
};

QString adapterName(PoseAdapterKind adapter);
AdapterResult parseRobotPoseLine(const QString &line, PoseAdapterKind adapter, int lineNumber);

} // namespace handeye
