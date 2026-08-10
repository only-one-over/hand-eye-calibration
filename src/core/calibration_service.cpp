#include "core/calibration_service.h"

#include "core/dataset_validator.h"
#include "core/matrix_utils.h"
#include "core/pose_quality_service.h"

#include <opencv2/calib3d.hpp>

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace handeye {

namespace {

cv::HandEyeCalibrationMethod toOpenCvMethod(CalibrationMethod method)
{
    switch (method) {
    case CalibrationMethod::Tsai: return cv::CALIB_HAND_EYE_TSAI;
    case CalibrationMethod::Park: return cv::CALIB_HAND_EYE_PARK;
    case CalibrationMethod::Horaud: return cv::CALIB_HAND_EYE_HORAUD;
    case CalibrationMethod::Andreff: return cv::CALIB_HAND_EYE_ANDREFF;
    case CalibrationMethod::Daniilidis: return cv::CALIB_HAND_EYE_DANIILIDIS;
    }
    return cv::CALIB_HAND_EYE_TSAI;
}

struct PairError {
    int first = 0;
    int second = 0;
    double rotationDeg = 0.0;
    double translationM = 0.0;
};

QVector<PairError> computePairErrors(const QVector<PoseSample> &samples, const cv::Matx44d &cameraToGripper)
{
    QVector<PairError> errors;
    for (int i = 0; i < samples.size(); ++i) {
        for (int j = i + 1; j < samples.size(); ++j) {
            const cv::Matx44d gripperI = matrix::fromRodrigues(samples[i].gripperRotation,
                                                                samples[i].gripperTranslation);
            const cv::Matx44d gripperJ = matrix::fromRodrigues(samples[j].gripperRotation,
                                                                samples[j].gripperTranslation);
            const cv::Matx44d targetI = matrix::fromRodrigues(samples[i].targetRotation,
                                                               samples[i].targetTranslation);
            const cv::Matx44d targetJ = matrix::fromRodrigues(samples[j].targetRotation,
                                                               samples[j].targetTranslation);
            // OpenCV's documented input convention gives A X = X B.
            const cv::Matx44d motionA = matrix::inverse(gripperJ) * gripperI;
            const cv::Matx44d motionB = targetJ * matrix::inverse(targetI);
            const cv::Matx44d error = matrix::inverse(motionA * cameraToGripper)
                                      * (cameraToGripper * motionB);
            const cv::Matx33d errorRotation = error.get_minor<3, 3>(0, 0);
            const double translation = std::sqrt(error(0, 3) * error(0, 3)
                                                 + error(1, 3) * error(1, 3)
                                                 + error(2, 3) * error(2, 3));
            errors.append({i, j, matrix::rotationAngleDeg(errorRotation), translation});
        }
    }
    return errors;
}

AxXbReport evaluate(const CalibrationDataset &dataset, const QVector<PoseSample> &samples,
                    const cv::Matx44d &cameraToGripper)
{
    AxXbReport report;
    report.available = true;
    report.sampleCount = samples.size();
    const ValidationReport validation = validateDataset(dataset, &samples);
    report.valid = validation.valid;
    report.errors = validation.errors;
    report.warnings = validation.warnings;
    if (!report.valid) return report;

    const QVector<PairError> pairErrors = computePairErrors(samples, cameraToGripper);
    if (pairErrors.isEmpty()) {
        report.valid = false;
        report.errors << QStringLiteral("无法形成相对运动残差。");
        return report;
    }

    double rotationSquared = 0.0;
    double translationSquared = 0.0;
    double rotationSum = 0.0;
    double translationSum = 0.0;
    QVector<double> rotationPerSample(samples.size(), 0.0);
    QVector<double> translationPerSample(samples.size(), 0.0);
    QVector<int> countPerSample(samples.size(), 0);
    for (const PairError &error : pairErrors) {
        rotationSquared += error.rotationDeg * error.rotationDeg;
        translationSquared += error.translationM * error.translationM;
        rotationSum += error.rotationDeg;
        translationSum += error.translationM;
        rotationPerSample[error.first] += error.rotationDeg * error.rotationDeg;
        rotationPerSample[error.second] += error.rotationDeg * error.rotationDeg;
        translationPerSample[error.first] += error.translationM * error.translationM;
        translationPerSample[error.second] += error.translationM * error.translationM;
        ++countPerSample[error.first];
        ++countPerSample[error.second];
    }

    const double pairCount = static_cast<double>(pairErrors.size());
    report.rotationRmseDeg = std::sqrt(rotationSquared / pairCount);
    report.translationRmseM = std::sqrt(translationSquared / pairCount);
    report.rotationMeanDeg = rotationSum / pairCount;
    report.translationMeanM = translationSum / pairCount;
    for (const PairError &error : pairErrors) {
        report.rotationMaxDeg = std::max(report.rotationMaxDeg, error.rotationDeg);
        report.translationMaxM = std::max(report.translationMaxM, error.translationM);
    }

    const double rotationLimit = std::max(dataset.passRotationRmseDeg, 1e-9);
    const double translationLimit = std::max(dataset.passTranslationRmseM, 1e-12);
    for (int index = 0; index < samples.size(); ++index) {
        SampleResidual residual;
        residual.sampleId = samples.at(index).id;
        residual.pairCount = countPerSample[index];
        if (residual.pairCount > 0) {
            residual.rotationErrorDeg = std::sqrt(rotationPerSample[index] / residual.pairCount);
            residual.translationErrorM = std::sqrt(translationPerSample[index] / residual.pairCount);
        }
        residual.normalizedScore = std::sqrt(std::pow(residual.rotationErrorDeg / rotationLimit, 2.0)
                                              + std::pow(residual.translationErrorM / translationLimit, 2.0));
        residual.outlier = residual.normalizedScore > 3.0;
        report.outlierCount += residual.outlier ? 1 : 0;
        report.sampleResiduals.append(residual);
    }
    report.passed = report.valid && report.rotationRmseDeg <= dataset.passRotationRmseDeg
                    && report.translationRmseM <= dataset.passTranslationRmseM
                    && report.outlierCount == 0;
    if (!report.passed)
        report.warnings << QStringLiteral("残差未达到通过阈值：旋转 %1°、平移 %2 m。")
                            .arg(report.rotationRmseDeg, 0, 'f', 6)
                            .arg(report.translationRmseM, 0, 'f', 9);
    return report;
}

double recommendationScore(const CalibrationDataset &dataset, const CalibrationResult &result)
{
    const AxXbReport &report = result.validationReport.available ? result.validationReport
                                                                  : result.axXbReport;
    return std::pow(report.rotationRmseDeg / std::max(dataset.passRotationRmseDeg, 1e-9), 2.0)
           + std::pow(report.translationRmseM / std::max(dataset.passTranslationRmseM, 1e-12), 2.0);
}

} // namespace

AxXbReport CalibrationService::evaluateAxXb(const CalibrationDataset &dataset,
                                            const Matrix4 &cameraToGripper,
                                            const QVector<PoseSample> &samples)
{
    const QVector<PoseSample> &evaluationSamples = samples.isEmpty() ? dataset.samples : samples;
    return evaluate(dataset, evaluationSamples, matrix::toMat(cameraToGripper));
}

CalibrationResult CalibrationService::calibrate(const CalibrationDataset &dataset, CalibrationMethod method)
{
    CalibrationResult result;
    result.method = method;
    result.seedMethod = method;
    QElapsedTimer timer;
    timer.start();

    const ValidationReport validation = validateDataset(dataset);
    if (!validation.valid) {
        result.message = validation.errors.join(QStringLiteral(" "));
        result.axXbReport.errors = validation.errors;
        result.axXbReport.warnings = validation.warnings;
        result.elapsedMs = timer.elapsed();
        return result;
    }

    try {
        std::vector<cv::Mat> gripperRotations;
        std::vector<cv::Mat> gripperTranslations;
        std::vector<cv::Mat> targetRotations;
        std::vector<cv::Mat> targetTranslations;
        for (const PoseSample &sample : dataset.samples) {
            const cv::Matx44d gripper = matrix::fromRodrigues(sample.gripperRotation, sample.gripperTranslation);
            const cv::Matx44d target = matrix::fromRodrigues(sample.targetRotation, sample.targetTranslation);
            gripperRotations.emplace_back(cv::Mat(gripper.get_minor<3, 3>(0, 0)).clone());
            gripperTranslations.emplace_back((cv::Mat_<double>(3, 1) << gripper(0, 3), gripper(1, 3), gripper(2, 3)));
            targetRotations.emplace_back(cv::Mat(target.get_minor<3, 3>(0, 0)).clone());
            targetTranslations.emplace_back((cv::Mat_<double>(3, 1) << target(0, 3), target(1, 3), target(2, 3)));
        }

        cv::Mat rotationOutput;
        cv::Mat translationOutput;
        cv::calibrateHandEye(gripperRotations, gripperTranslations, targetRotations, targetTranslations,
                             rotationOutput, translationOutput, toOpenCvMethod(method));

        cv::Mat rotation64;
        rotationOutput.convertTo(rotation64, CV_64F);
        cv::Matx33d rotation;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                rotation(row, col) = rotation64.at<double>(row, col);
        const cv::Vec3d translation(translationOutput.at<double>(0), translationOutput.at<double>(1),
                                    translationOutput.at<double>(2));
        const cv::Matx44d cameraToGripper = matrix::fromRodrigues(matrix::toRodrigues(rotation),
                                                                   {translation[0], translation[1], translation[2]});
        result.cameraToGripper = matrix::toArray(cameraToGripper);
        result.axXbReport = evaluate(dataset, dataset.samples, cameraToGripper);
        result.trainingReport = result.axXbReport;
        if (dataset.validationSamples.size() >= 3)
            result.validationReport = evaluate(dataset, dataset.validationSamples, cameraToGripper);
        result.rotationErrorDeg = result.axXbReport.rotationRmseDeg;
        result.translationError = result.axXbReport.translationRmseM;
        result.fixedTargetReport = PoseQualityService::computeFixedTargetPose(dataset, result.cameraToGripper);
        result.qualityReport = PoseQualityService::evaluatePoseQuality(dataset);
        result.success = true;
        result.message = result.axXbReport.passed ? QStringLiteral("计算成功，训练数据通过")
                                                       : QStringLiteral("计算成功，但可靠性未通过");
    } catch (const cv::Exception &error) {
        result.message = QStringLiteral("OpenCV 错误：%1").arg(QString::fromStdString(error.what()));
    } catch (const std::exception &error) {
        result.message = QStringLiteral("计算错误：%1").arg(QString::fromStdString(error.what()));
    }

    result.elapsedMs = timer.elapsed();
    return result;
}

QVector<CalibrationResult> CalibrationService::calibrateAll(const CalibrationDataset &dataset)
{
    QVector<CalibrationResult> results;
    for (CalibrationMethod method : allMethods())
        results.append(calibrate(dataset, method));

    int recommendedIndex = -1;
    bool anyPassing = false;
    double bestScore = std::numeric_limits<double>::max();
    for (int index = 0; index < results.size(); ++index) {
        const CalibrationResult &result = results.at(index);
        const bool passing = result.success && result.axXbReport.passed
                             && (!result.validationReport.available || result.validationReport.passed);
        if (passing && !anyPassing) {
            anyPassing = true;
            bestScore = std::numeric_limits<double>::max();
        }
        if (result.success && (passing == anyPassing)) {
            const double score = recommendationScore(dataset, result);
            if (score < bestScore) {
                bestScore = score;
                recommendedIndex = index;
            }
        }
    }
    if (recommendedIndex >= 0) {
        results[recommendedIndex].recommended = true;
        results[recommendedIndex].message += QStringLiteral("；推荐结果");
    }
    return results;
}

} // namespace handeye
