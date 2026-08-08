#include "core/calibration_service.h"

#include "core/dataset_validator.h"
#include "core/matrix_utils.h"

#include <opencv2/calib3d.hpp>

#include <QElapsedTimer>

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

double frobenius(const cv::Matx33d &matrix)
{
    double sum = 0.0;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            sum += matrix(row, col) * matrix(row, col);
    return std::sqrt(sum);
}

} // namespace

CalibrationResult CalibrationService::calibrate(const CalibrationDataset &dataset, CalibrationMethod method)
{
    CalibrationResult result;
    result.method = method;
    QElapsedTimer timer;
    timer.start();

    const ValidationReport validation = validateDataset(dataset);
    if (!validation.valid) {
        result.message = validation.errors.join(QStringLiteral(" "));
        result.elapsedMs = timer.elapsed();
        return result;
    }

    try {
        std::vector<cv::Mat> gripperRotations;
        std::vector<cv::Mat> gripperTranslations;
        std::vector<cv::Mat> targetRotations;
        std::vector<cv::Mat> targetTranslations;
        gripperRotations.reserve(dataset.samples.size());
        gripperTranslations.reserve(dataset.samples.size());
        targetRotations.reserve(dataset.samples.size());
        targetTranslations.reserve(dataset.samples.size());

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

        cv::Matx33d rotation;
        rotationOutput.convertTo(rotation, CV_64F);
        const cv::Vec3d translation(translationOutput.at<double>(0), translationOutput.at<double>(1),
                                    translationOutput.at<double>(2));
        const cv::Matx44d cameraToGripper = matrix::fromRodrigues(matrix::toRodrigues(rotation),
                                                                   {translation[0], translation[1], translation[2]});
        result.cameraToGripper = matrix::toArray(cameraToGripper);
        result.success = true;
        result.message = QStringLiteral("计算成功");

        double rotationResidual = 0.0;
        double translationResidual = 0.0;
        int residualCount = 0;
        for (int i = 0; i < dataset.samples.size(); ++i) {
            for (int j = i + 1; j < dataset.samples.size(); ++j) {
                const cv::Matx44d gripperI = matrix::fromRodrigues(dataset.samples[i].gripperRotation,
                                                                     dataset.samples[i].gripperTranslation);
                const cv::Matx44d gripperJ = matrix::fromRodrigues(dataset.samples[j].gripperRotation,
                                                                     dataset.samples[j].gripperTranslation);
                const cv::Matx44d targetI = matrix::fromRodrigues(dataset.samples[i].targetRotation,
                                                                    dataset.samples[i].targetTranslation);
                const cv::Matx44d targetJ = matrix::fromRodrigues(dataset.samples[j].targetRotation,
                                                                    dataset.samples[j].targetTranslation);
                const cv::Matx44d motionA = matrix::inverse(gripperJ) * gripperI;
                const cv::Matx44d motionB = targetJ * matrix::inverse(targetI);
                const cv::Matx44d error = matrix::inverse(motionA * cameraToGripper)
                                          * (cameraToGripper * motionB);
                rotationResidual += matrix::rotationAngleDeg(error.get_minor<3, 3>(0, 0));
                translationResidual += frobenius(error.get_minor<3, 3>(0, 0)) * 0.0
                                       + std::sqrt(error(0, 3) * error(0, 3) + error(1, 3) * error(1, 3)
                                                   + error(2, 3) * error(2, 3));
                ++residualCount;
            }
        }
        if (residualCount > 0) {
            result.rotationErrorDeg = rotationResidual / residualCount;
            result.translationError = translationResidual / residualCount;
        }
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
    return results;
}

} // namespace handeye
