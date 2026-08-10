#include "core/normalized_huber.h"

#include <algorithm>
#include <cmath>

namespace handeye {

namespace {

NormalizedHuberEvaluation evaluateNorm(double norm, double delta)
{
    NormalizedHuberEvaluation result;
    result.norm = norm;
    result.outlier = norm > delta;
    if (norm <= delta) {
        result.loss = 0.5 * norm * norm;
        result.weight = 1.0;
    } else {
        result.loss = delta * (norm - 0.5 * delta);
        result.weight = delta / std::max(norm, 1e-12);
    }
    return result;
}

} // namespace

NormalizedHuberEvaluation NormalizedHuber::evaluate(const Vector3 &rotationResidual,
                                                    const Vector3 &translationResidual,
                                                    const NormalizedHuberConfig &config)
{
    const double rotationScale = std::max(std::abs(config.rotationScaleRad), 1e-12);
    const double translationScale = std::max(std::abs(config.translationScaleM), 1e-12);
    double squared = 0.0;
    for (double value : rotationResidual) squared += std::pow(value / rotationScale, 2.0);
    for (double value : translationResidual) squared += std::pow(value / translationScale, 2.0);
    return evaluateNorm(std::sqrt(squared), std::max(config.delta, 1e-12));
}

NormalizedHuberEvaluation NormalizedHuber::evaluateTranslation(const Vector3 &translationResidual,
                                                               const NormalizedHuberConfig &config)
{
    const double scale = std::max(std::abs(config.translationScaleM), 1e-12);
    double squared = 0.0;
    for (double value : translationResidual) squared += std::pow(value / scale, 2.0);
    return evaluateNorm(std::sqrt(squared), std::max(config.delta, 1e-12));
}

} // namespace handeye
