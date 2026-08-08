#include "mainwindow.h"

#include "core/calibration_service.h"
#include "core/dataset_validator.h"
#include "core/pose_conversion.h"
#include "core/synthetic_dataset.h"
#include "io/dataset_io.h"

#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QTemporaryDir>
#include <QLocale>
#include <QTranslator>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>

namespace {

double matrixMaxError(const handeye::Matrix4 &left, const handeye::Matrix4 &right)
{
    double result = 0.0;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            result = std::max(result, std::abs(left[row][col] - right[row][col]));
    return result;
}

} // namespace

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QTranslator translator;
    if (translator.load(QLocale::system(), QStringLiteral("hand_eye_calibration"), QStringLiteral("_"),
                       QStringLiteral(":/i18n")))
        app.installTranslator(&translator);
    if (app.arguments().contains(QStringLiteral("--smoke-test"))) {
        QFile smokeLog(QStringLiteral("smoke-result.txt"));
        smokeLog.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream smokeStream(&smokeLog);

        const handeye::CalibrationDataset dataset = handeye::makeSyntheticDataset();
        const QVector<handeye::CalibrationResult> results = handeye::CalibrationService::calibrateAll(dataset);
        int successCount = 0;
        int recommendedCount = 0;
        double maxTruthError = 0.0;
        for (const handeye::CalibrationResult &result : results) {
            successCount += result.success ? 1 : 0;
            recommendedCount += result.recommended ? 1 : 0;
            if (result.success && dataset.hasGroundTruth)
                maxTruthError = std::max(maxTruthError, matrixMaxError(result.cameraToGripper,
                                                                         dataset.groundTruthCameraToGripper));
            smokeStream << handeye::methodName(result.method) << "|" << result.success << "|"
                        << result.message << "|rmse=" << result.trainingReport.rotationRmseDeg
                        << "," << result.trainingReport.translationRmseM << Qt::endl;
        }

        const auto degreeMm = handeye::pose::normalize({0.0, 0.0, 90.0, 0.0}, {1000.0, 0.0, 0.0},
                                                       {handeye::RotationFormat::Rodrigues,
                                                        handeye::AngleUnit::Degrees,
                                                        handeye::LengthUnit::Millimeters});
        const bool normalizationOk = degreeMm.success
                                      && std::abs(degreeMm.rotation[2] - CV_PI / 2.0) < 1e-10
                                      && std::abs(degreeMm.translation[0] - 1.0) < 1e-10;

        handeye::CalibrationDataset validationDataset = dataset;
        validationDataset.validationSamples = handeye::makeSyntheticDataset(6).samples;
        const auto validationResult = handeye::CalibrationService::calibrate(validationDataset,
                                                                              handeye::CalibrationMethod::Tsai);
        const bool independentValidationOk = validationResult.validationReport.available
                                             && validationResult.validationReport.passed;

        handeye::CalibrationDataset outlierDataset = handeye::makeSyntheticDataset();
        outlierDataset.samples.last().targetTranslation[0] += 0.2;
        const auto outlierResult = handeye::CalibrationService::calibrate(outlierDataset,
                                                                           handeye::CalibrationMethod::Tsai);
        const bool outlierDetectionOk = outlierResult.trainingReport.outlierCount > 0;

        handeye::CalibrationDataset degenerateDataset = dataset;
        for (handeye::PoseSample &sample : degenerateDataset.samples)
            sample.gripperRotation = {0.0, 0.0, 0.0};
        const bool degeneracyOk = !handeye::validateDataset(degenerateDataset).valid;

        QTemporaryDir tempDir;
        handeye::CalibrationDataset csvDataset;
        handeye::CalibrationDataset jsonDataset;
        const auto csvWrite = handeye::writeCsv(tempDir.filePath(QStringLiteral("samples.csv")), dataset);
        const auto csvRead = handeye::readCsv(tempDir.filePath(QStringLiteral("samples.csv")), &csvDataset,
                                              handeye::PoseInputSpec{});
        const auto jsonWrite = handeye::writeJson(tempDir.filePath(QStringLiteral("samples.json")), dataset);
        const auto jsonRead = handeye::readJson(tempDir.filePath(QStringLiteral("samples.json")), &jsonDataset);
        const auto yamlWrite = handeye::writeYaml(tempDir.filePath(QStringLiteral("result.yaml")), dataset);
        const auto txtWrite = results.isEmpty() ? handeye::IoResult{false, {}}
                                                 : handeye::writeResultTxt(tempDir.filePath(QStringLiteral("result.txt")), dataset, results.first());
        const auto cppWrite = results.isEmpty() ? handeye::IoResult{false, {}}
                                                 : handeye::writeResultCpp(tempDir.filePath(QStringLiteral("result.cpp")), dataset, results.first());
        const auto pythonWrite = results.isEmpty() ? handeye::IoResult{false, {}}
                                                   : handeye::writeResultPython(tempDir.filePath(QStringLiteral("result.py")), dataset, results.first());
        const bool ioOk = csvWrite.success && csvRead.success && jsonWrite.success && jsonRead.success
                          && yamlWrite.success && txtWrite.success && cppWrite.success && pythonWrite.success
                          && csvDataset.samples.size() == dataset.samples.size()
                          && jsonDataset.samples.size() == dataset.samples.size();

        const bool allOk = successCount == handeye::allMethods().size()
                           && recommendedCount == 1
                           && maxTruthError < 1e-5
                           && normalizationOk && independentValidationOk
                           && outlierDetectionOk && degeneracyOk && ioOk;
        smokeStream << "success=" << successCount << "/" << handeye::allMethods().size() << Qt::endl;
        smokeStream << "recommended=" << recommendedCount << ",truth_max_error=" << maxTruthError << Qt::endl;
        smokeStream << "normalization=" << normalizationOk << ",independent_validation=" << independentValidationOk
                    << ",outlier_detection=" << outlierDetectionOk << ",degeneracy=" << degeneracyOk << Qt::endl;
        smokeStream << "io_roundtrip_and_exports=" << ioOk << Qt::endl;
        qInfo() << "smoke-test" << allOk;
        return allOk ? 0 : 1;
    }
    handeye::MainWindow window;
    window.show();
    return app.exec();
}
