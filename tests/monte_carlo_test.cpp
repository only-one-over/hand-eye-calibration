#include "core/calibration_service.h"
#include "core/matrix_utils.h"
#include "core/nonlinear_optimizer.h"
#include "core/reliability_pipeline_service.h"

#include <opencv2/core.hpp>

#include <QtTest>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

using namespace handeye;

namespace {

constexpr double kPi = 3.14159265358979323846;

double gaussian(std::mt19937 &generator, double sigma)
{
    std::normal_distribution<double> distribution(0.0, sigma);
    return distribution(generator);
}

cv::Matx44d noisyPose(const cv::Matx44d &pose, std::mt19937 &generator,
                      double rotationSigma, double translationSigma)
{
    Vector3 rotation = matrix::toRodrigues(pose.get_minor<3, 3>(0, 0));
    Vector3 translation{pose(0, 3), pose(1, 3), pose(2, 3)};
    for (int axis = 0; axis < 3; ++axis) {
        rotation[axis] += gaussian(generator, rotationSigma);
        translation[axis] += gaussian(generator, translationSigma);
    }
    return matrix::fromRodrigues(rotation, translation);
}

CalibrationDataset makeNoisyDataset(std::mt19937 &generator)
{
    CalibrationDataset dataset;
    dataset.targetPosesReady = true;
    dataset.passRotationRmseDeg = 2.0;
    dataset.passTranslationRmseM = 0.01;
    const cv::Matx44d truth = matrix::fromRodrigues({0.18, -0.22, 0.12}, {0.08, -0.04, 0.16});

    for (int index = 0; index < 20; ++index) {
        const double phase = static_cast<double>(index + 1);
        const cv::Matx44d robot = matrix::fromRodrigues(
            {0.65 * std::sin(phase * 0.71), 0.55 * std::cos(phase * 0.53),
             0.45 * std::sin(phase * 0.37)},
            {0.10 * std::sin(phase * 0.41), 0.08 * std::cos(phase * 0.29),
             0.30 + 0.06 * std::sin(phase * 0.67)});
        const cv::Matx44d target = matrix::inverse(truth) * matrix::inverse(robot);
        cv::Matx44d noisyRobot = noisyPose(robot, generator, 0.0005, 0.0005);
        cv::Matx44d noisyTarget = noisyPose(target, generator, 0.001, 0.0008);

        PoseSample sample;
        sample.id = index + 1;
        sample.imagePath = QStringLiteral("synthetic_%1.png").arg(index + 1);
        sample.imageStatus = ImageSampleStatus::PoseEstimated;
        sample.pnpReprojectionRmsePx = 0.35;
        if (index % 10 == 0) {
            Vector3 outlierRotation = matrix::toRodrigues(noisyTarget.get_minor<3, 3>(0, 0));
            Vector3 outlierTranslation{noisyTarget(0, 3), noisyTarget(1, 3), noisyTarget(2, 3)};
            outlierRotation[0] += 0.16;
            outlierRotation[1] -= 0.11;
            outlierTranslation[0] += 0.035;
            outlierTranslation[1] -= 0.025;
            noisyTarget = matrix::fromRodrigues(outlierRotation, outlierTranslation);
            sample.pnpReprojectionRmsePx = 12.0;
        }
        sample.gripperRotation = matrix::toRodrigues(noisyRobot.get_minor<3, 3>(0, 0));
        sample.gripperTranslation = {noisyRobot(0, 3), noisyRobot(1, 3), noisyRobot(2, 3)};
        sample.targetRotation = matrix::toRodrigues(noisyTarget.get_minor<3, 3>(0, 0));
        sample.targetTranslation = {noisyTarget(0, 3), noisyTarget(1, 3), noisyTarget(2, 3)};
        dataset.samples.append(sample);
    }
    return dataset;
}

double rotationErrorDeg(const Matrix4 &estimated, const cv::Matx44d &truth)
{
    const cv::Matx44d error = matrix::inverse(truth) * matrix::toMat(estimated);
    return matrix::rotationAngleDeg(error.get_minor<3, 3>(0, 0));
}

double translationErrorM(const Matrix4 &estimated, const cv::Matx44d &truth)
{
    const cv::Matx44d error = matrix::inverse(truth) * matrix::toMat(estimated);
    return std::sqrt(error(0, 3) * error(0, 3) + error(1, 3) * error(1, 3)
                     + error(2, 3) * error(2, 3));
}

} // namespace

class MonteCarloTest : public QObject
{
    Q_OBJECT

private slots:
    void noisyPosePairsWithOutliersRemainSolvable();
};

void MonteCarloTest::noisyPosePairsWithOutliersRemainSolvable()
{
    constexpr int trialCount = 100;
    std::mt19937 generator(0x20260810u);
    QVector<double> rotationErrors;
    QVector<double> translationErrors;
    int nonlinearSuccesses = 0;

    for (int trial = 0; trial < trialCount; ++trial) {
        CalibrationDataset dataset = makeNoisyDataset(generator);
        const cv::Matx44d truth = matrix::fromRodrigues({0.18, -0.22, 0.12}, {0.08, -0.04, 0.16});
        const CalibrationResult raw = CalibrationService::calibrate(dataset, CalibrationMethod::Tsai);
        QVERIFY2(raw.success, qPrintable(QStringLiteral("raw trial %1: %2").arg(trial).arg(raw.message)));
        // The production default remains 50 iterations. Monte-Carlo keeps a
        // bounded two-step refinement so 100 deterministic trials stay suitable
        // for Debug CI while still exercising the exact production optimizer.
        const CalibrationResult refined = NonlinearOptimizer::refinePose(dataset, raw, 2);
        if (!refined.success) continue;
        ++nonlinearSuccesses;
        rotationErrors.append(rotationErrorDeg(refined.cameraToGripper, truth));
        translationErrors.append(translationErrorM(refined.cameraToGripper, truth));
    }

    QVERIFY(nonlinearSuccesses >= 90);
    std::sort(rotationErrors.begin(), rotationErrors.end());
    std::sort(translationErrors.begin(), translationErrors.end());
    QVERIFY(!rotationErrors.isEmpty());
    QVERIFY(!translationErrors.isEmpty());
    const int median = rotationErrors.size() / 2;
    QVERIFY2(rotationErrors.at(median) <= 1.0,
             qPrintable(QStringLiteral("median rotation error=%1 deg").arg(rotationErrors.at(median))));
    QVERIFY2(translationErrors.at(median) <= 0.005,
             qPrintable(QStringLiteral("median translation error=%1 m").arg(translationErrors.at(median))));

    std::mt19937 pipelineGenerator(0x20260810u);
    const CalibrationDataset pipelineDataset = makeNoisyDataset(pipelineGenerator);
    const CalibrationResult raw = CalibrationService::calibrate(pipelineDataset, CalibrationMethod::Tsai);
    const ReliabilityPipelineExecution execution = ReliabilityPipelineService::run(
        pipelineDataset, 10, 0.95);
    QVERIFY(execution.report.pnpReport.available);
    QVERIFY(execution.report.pnpReport.outlierCount >= 2);
    QVERIFY(!execution.report.candidateSampleIds.isEmpty()
            || !execution.report.removedSampleIds.isEmpty()
            || !execution.report.outlierValidation.isEmpty());
    QVERIFY2(execution.finalResult.success, qPrintable(execution.report.message));
    QVERIFY(execution.finalResult.axXbReport.translationRmseM <= raw.axXbReport.translationRmseM
            || execution.report.autoRemovedCount > 0);
}

QTEST_MAIN(MonteCarloTest)

#include "monte_carlo_test.moc"
