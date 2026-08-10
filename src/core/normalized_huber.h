#pragma once

#include "domain/calibration_types.h"

namespace handeye {

struct NormalizedHuberConfig {
    double rotationScaleRad = 3.14159265358979323846 / 180.0;
    double translationScaleM = 0.001;
    double delta = 1.0;
};

struct NormalizedHuberEvaluation {
    double norm = 0.0;
    double loss = 0.0;
    double weight = 1.0;
    bool outlier = false;
};

class NormalizedHuber
{
public:
    static NormalizedHuberEvaluation evaluate(const Vector3 &rotationResidual,
                                              const Vector3 &translationResidual,
                                              const NormalizedHuberConfig &config = {});
    static NormalizedHuberEvaluation evaluateTranslation(const Vector3 &translationResidual,
                                                         const NormalizedHuberConfig &config = {});
};

} // namespace handeye
