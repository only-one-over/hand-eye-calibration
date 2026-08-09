#include "mainwindow.h"

#include "core/board_pose_estimator.h"
#include "core/camera_calibration_service.h"
#include "core/calibration_service.h"
#include "core/dataset_validator.h"
#include "core/matrix_utils.h"
#include "core/pose_conversion.h"
#include "core/nonlinear_optimizer.h"
#include "core/point_calibration_service.h"
#include "core/pose_quality_service.h"
#include "core/reliability_pipeline_service.h"
#include "core/synthetic_dataset.h"
#include "controllers/calibration_controller.h"
#include "io/dataset_io.h"
#include "io/image_sample_io.h"
#include "views/calibration_result_page.h"
#include "views/camera_calibration_page.h"
#include "views/current_data_page.h"
#include "views/manual_pose_page.h"

#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QTemporaryDir>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QTableView>
#include <QTableWidget>
#include <QTabWidget>
#include <QLocale>
#include <QTranslator>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

double matrixMaxError(const handeye::Matrix4 &left, const handeye::Matrix4 &right)
{
    double result = 0.0;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            result = std::max(result, std::abs(left[row][col] - right[row][col]));
    return result;
}

double matrix3MaxError(const handeye::Matrix3 &left, const handeye::Matrix3 &right)
{
    double result = 0.0;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            result = std::max(result, std::abs(left[row][col] - right[row][col]));
    return result;
}

handeye::Vector3 transformPoint(const cv::Matx44d &pose, const handeye::Vector3 &point)
{
    return {pose(0, 0) * point[0] + pose(0, 1) * point[1] + pose(0, 2) * point[2] + pose(0, 3),
            pose(1, 0) * point[0] + pose(1, 1) * point[1] + pose(1, 2) * point[2] + pose(1, 3),
            pose(2, 0) * point[0] + pose(2, 1) * point[1] + pose(2, 2) * point[2] + pose(2, 3)};
}

QVector<handeye::CameraCalibrationSample> makeSyntheticCameraObservations(
    const handeye::BoardSpec &board,
    const handeye::Matrix3 &cameraMatrix,
    const handeye::Vector5 &distortion,
    int count)
{
    cv::Mat cvCameraMatrix(3, 3, CV_64F);
    cv::Mat cvDistortion(1, 5, CV_64F);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            cvCameraMatrix.at<double>(row, col) = cameraMatrix[row][col];
    for (int index = 0; index < 5; ++index) cvDistortion.at<double>(0, index) = distortion[index];

    std::vector<cv::Point3f> objectPoints;
    for (int row = 0; row < board.innerCornersY; ++row)
        for (int col = 0; col < board.innerCornersX; ++col)
            objectPoints.emplace_back(static_cast<float>(col * board.squareSizeM),
                                      static_cast<float>(row * board.squareSizeM), 0.0F);

    QVector<handeye::CameraCalibrationSample> samples;
    for (int index = 0; index < count; ++index) {
        const cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.03 * std::sin(index * 0.7),
                              -0.18 + 0.025 * index, 0.04 * std::cos(index * 0.5));
        const cv::Mat tvec = (cv::Mat_<double>(3, 1) << -0.11 + 0.018 * index,
                              -0.05 + 0.022 * (index % 5), 0.75 + 0.035 * (index % 4));
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objectPoints, rvec, tvec, cvCameraMatrix, cvDistortion, projected);

        handeye::CameraCalibrationSample sample;
        sample.imagePath = QStringLiteral("synthetic_camera_%1.png").arg(index + 1);
        sample.imageWidth = 640;
        sample.imageHeight = 480;
        sample.status = handeye::CameraCalibrationSampleStatus::Valid;
        sample.used = true;
        for (const cv::Point2f &point : projected) sample.corners.append({point.x, point.y});
        sample.detectedCornerCount = sample.corners.size();
        samples.append(sample);
    }
    return samples;
}

} // namespace

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QFile styleFile(QStringLiteral(":/assets/style.qss"));
    if (styleFile.open(QIODevice::ReadOnly | QIODevice::Text))
        app.setStyleSheet(QString::fromUtf8(styleFile.readAll()));

    QTranslator translator;
    if (translator.load(QLocale::system(), QStringLiteral("hand_eye_calibration"), QStringLiteral("_"),
                       QStringLiteral(":/i18n")))
        app.installTranslator(&translator);
    if (app.arguments().contains(QStringLiteral("--pipeline-test"))) {
        const handeye::CalibrationDataset dataset = handeye::makeSyntheticDataset();
        const auto execution = handeye::ReliabilityPipelineService::run(dataset, 1, 0.95);
        qInfo() << "pipeline-test" << execution.report.success
                << execution.report.bootstrapReport.success
                << execution.report.message;
        return execution.report.success && execution.report.bootstrapReport.success ? 0 : 1;
    }
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

        handeye::CalibrationDataset pointDataset;
        pointDataset.inputMode = handeye::CalibrationInputMode::FixedPoint3D;
        const cv::Matx44d pointTruth = handeye::matrix::toMat(dataset.groundTruthCameraToGripper);
        const handeye::Vector3 fixedBasePoint{0.25, -0.12, 0.55};
        for (const handeye::PoseSample &poseSample : dataset.samples) {
            const cv::Matx44d gripper = handeye::matrix::fromRodrigues(poseSample.gripperRotation,
                                                                        poseSample.gripperTranslation);
            const cv::Matx44d cameraPointPose = handeye::matrix::inverse(pointTruth)
                                                * handeye::matrix::inverse(gripper);
            handeye::PointSample pointSample;
            pointSample.id = poseSample.id;
            pointSample.gripperRotation = poseSample.gripperRotation;
            pointSample.gripperTranslation = poseSample.gripperTranslation;
            pointSample.cameraPoint = transformPoint(cameraPointPose, fixedBasePoint);
            pointDataset.pointSamples.append(pointSample);
        }
        const auto pointResult = handeye::PointCalibrationService::calibrate(pointDataset);
        const bool pointCalibrationOk = pointResult.success
                                        && matrixMaxError(pointResult.cameraToGripper,
                                                          dataset.groundTruthCameraToGripper) < 1e-4
                                        && pointResult.fixedPointReport.rmseM < 1e-6;
        const auto fixedTargetReport = handeye::PoseQualityService::computeFixedTargetPose(
            dataset, results.first().cameraToGripper);
        const auto qualityReport = handeye::PoseQualityService::evaluatePoseQuality(dataset);
        const auto nonlinearResult = handeye::NonlinearOptimizer::refinePose(dataset, results.first());
        const bool fixedTargetOk = fixedTargetReport.success && fixedTargetReport.samples.size() == dataset.samples.size();
        const bool qualityOk = qualityReport.available && qualityReport.totalScore > 0;
        const bool nonlinearOk = nonlinearResult.success
                                 && nonlinearResult.optimizationReport.afterTranslationRmseM
                                        <= nonlinearResult.optimizationReport.beforeTranslationRmseM + 1e-9;
        const auto pipelineExecution = handeye::ReliabilityPipelineService::run(dataset, 1, 0.95);
        const bool pipelineOk = pipelineExecution.report.available
                                && pipelineExecution.report.success
                                && pipelineExecution.report.bootstrapReport.success
                                && pipelineExecution.finalResult.success
                                && pipelineExecution.report.stages.size() >= 8
                                && pipelineExecution.report.finalSampleCount >= 5;
        handeye::CalibrationController pointController;
        const bool manualPointApplyOk = pointController.applyManualPointInputs(pointDataset.pointSamples,
                                                                                 handeye::PoseInputSpec{})
                                        && pointController.dataset().inputMode
                                               == handeye::CalibrationInputMode::FixedPoint3D
                                        && pointController.dataset().pointSamples.size()
                                               == pointDataset.pointSamples.size();

        handeye::CalibrationController manualController;
        handeye::PoseInputSpec manualSpec;
        QVector<handeye::ManualPoseInput> manualInputs;
        for (const handeye::PoseSample &sample : dataset.samples) {
            handeye::ManualPoseInput input;
            input.id = sample.id;
            input.tcpRotation = {sample.gripperRotation[0], sample.gripperRotation[1],
                                 sample.gripperRotation[2], 0.0};
            input.tcpTranslation = sample.gripperTranslation;
            input.cameraRotation = {sample.targetRotation[0], sample.targetRotation[1],
                                    sample.targetRotation[2], 0.0};
            input.cameraTranslation = sample.targetTranslation;
            manualInputs.append(input);
        }
        const bool manualApplyOk = manualController.applyManualPoseInputs(manualInputs, manualSpec)
                                   && manualController.dataset().targetPosesReady
                                   && manualController.dataset().samples.size() == dataset.samples.size()
                                   && manualController.dataset().samples.first().imageStatus
                                          == handeye::ImageSampleStatus::ManualPose;
        auto duplicateManualInputs = manualInputs;
        duplicateManualInputs.last().id = duplicateManualInputs.first().id;
        const bool manualDuplicateRejected = !manualController.applyManualPoseInputs(duplicateManualInputs,
                                                                                       manualSpec);
        handeye::PoseInputSpec quaternionSpec;
        quaternionSpec.rotationFormat = handeye::RotationFormat::QuaternionWXYZ;
        quaternionSpec.angleUnit = handeye::AngleUnit::Degrees;
        quaternionSpec.lengthUnit = handeye::LengthUnit::Millimeters;
        const auto quaternionNormalized = handeye::pose::normalize(
            {std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)}, {1000.0, 0.0, 0.0}, quaternionSpec);
        const bool manualQuaternionOk = quaternionNormalized.success
                                        && std::abs(quaternionNormalized.rotation[2] - CV_PI / 2.0) < 1e-10
                                        && std::abs(quaternionNormalized.translation[0] - 1.0) < 1e-10;

        QTemporaryDir imageDir;
        const int squarePixels = 60;
        const int marginPixels = 30;
        const int boardSquaresX = 10;
        const int boardSquaresY = 7;
        cv::Mat chessboard(boardSquaresY * squarePixels + marginPixels * 2,
                           boardSquaresX * squarePixels + marginPixels * 2, CV_8UC1,
                           cv::Scalar(255));
        for (int row = 0; row < boardSquaresY; ++row) {
            for (int col = 0; col < boardSquaresX; ++col) {
                if ((row + col) % 2 == 0) {
                    cv::rectangle(chessboard,
                                  cv::Rect(marginPixels + col * squarePixels,
                                           marginPixels + row * squarePixels,
                                           squarePixels, squarePixels),
                                  cv::Scalar(0), cv::FILLED);
                }
            }
        }
        const QString chessboardPath = imageDir.filePath(QStringLiteral("board_001.png"));
        const bool imageWriteOk = !imageDir.path().isEmpty()
                                  && cv::imwrite(chessboardPath.toLocal8Bit().constData(), chessboard);
        handeye::BoardSpec boardSpec;
        handeye::CameraIntrinsics intrinsics;
        intrinsics.valid = true;
        intrinsics.cameraMatrix = {{{800.0, 0.0, chessboard.cols / 2.0},
                                    {0.0, 800.0, chessboard.rows / 2.0},
                                    {0.0, 0.0, 1.0}}};
        const auto imagePose = handeye::BoardPoseEstimator::estimateChessboard(chessboardPath,
                                                                                 boardSpec, intrinsics);
        const bool imagePoseOk = imageWriteOk && imagePose.success
                                  && imagePose.detectedCornerCount == boardSpec.innerCornersX * boardSpec.innerCornersY
                                  && imagePose.reprojectionRmsePx < 1.0;

        const QString pairedCsvPath = imageDir.filePath(QStringLiteral("paired.csv"));
        QFile pairedFile(pairedCsvPath);
        pairedFile.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream pairedStream(&pairedFile);
        pairedStream << "id,image_path,tx,ty,tz,rx,ry,rz\n"
                     << "1," << chessboardPath << ",0,0,0,0,0,0\n";
        pairedFile.close();
        handeye::CalibrationDataset pairedDataset;
        const auto pairedRead = handeye::readPoseImageCsv(pairedCsvPath, &pairedDataset,
                                                           handeye::PoseInputSpec{});
        const bool pairedImportOk = pairedRead.success && pairedDataset.samples.size() == 1
                                    && pairedDataset.samples.first().imagePath == chessboardPath;

        handeye::BoardSpec cameraBoard;
        cameraBoard.pattern = handeye::BoardPattern::Chessboard;
        cameraBoard.innerCornersX = 9;
        cameraBoard.innerCornersY = 6;
        cameraBoard.squareSizeM = 0.025;
        const handeye::Matrix3 knownCameraMatrix{{{800.0, 0.0, 320.0},
                                                   {0.0, 820.0, 240.0},
                                                   {0.0, 0.0, 1.0}}};
        const handeye::Vector5 knownDistortion{-0.12, 0.03, 0.001, -0.002, -0.01};
        const auto cameraObservations = makeSyntheticCameraObservations(cameraBoard, knownCameraMatrix,
                                                                          knownDistortion, 12);
        const auto cameraReport = handeye::CameraCalibrationService::calibrateObservations(cameraObservations,
                                                                                              cameraBoard);
        const bool cameraCalibrationOk = cameraReport.success
                                         && cameraReport.finalUsedCount >= 6
                                         && matrix3MaxError(cameraReport.intrinsics.cameraMatrix, knownCameraMatrix) < 0.5
                                         && std::abs(cameraReport.intrinsics.distortionCoeffs[0] - knownDistortion[0]) < 0.02;
        const auto cameraOutlierObservations = [&cameraObservations] {
            auto result = cameraObservations;
            for (int index = 0; index < result.last().corners.size(); ++index) {
                if (index % 2 == 0) {
                    result.last().corners[index][0] += 18.0;
                    result.last().corners[index][1] -= 11.0;
                }
            }
            return result;
        }();
        const auto cameraOutlierReport = handeye::CameraCalibrationService::calibrateObservations(
            cameraOutlierObservations, cameraBoard);
        const bool cameraOutlierOk = cameraOutlierReport.success && cameraOutlierReport.outlierCount > 0
                                     && cameraOutlierReport.finalUsedCount >= 6;
        const auto cameraTooFewReport = handeye::CameraCalibrationService::calibrateObservations(
            cameraObservations.mid(0, 5), cameraBoard);
        const bool cameraTooFewOk = !cameraTooFewReport.success && !cameraTooFewReport.errors.isEmpty();
        auto mixedResolutionObservations = cameraObservations;
        mixedResolutionObservations.last().imageWidth = 800;
        const auto mixedResolutionReport = handeye::CameraCalibrationService::calibrateObservations(
            mixedResolutionObservations, cameraBoard);
        const bool cameraResolutionOk = !mixedResolutionReport.success
                                        && mixedResolutionReport.errors.join('\n').contains(QStringLiteral("分辨率"));

        QTemporaryDir tempDir;
        handeye::CalibrationDataset csvDataset;
        handeye::CalibrationDataset jsonDataset;
        const auto csvWrite = handeye::writeCsv(tempDir.filePath(QStringLiteral("samples.csv")), dataset);
        const auto csvRead = handeye::readCsv(tempDir.filePath(QStringLiteral("samples.csv")), &csvDataset,
                                              handeye::PoseInputSpec{});
        handeye::CalibrationDataset jsonSource = dataset;
        jsonSource.cameraIntrinsics = cameraReport.intrinsics;
        jsonSource.cameraCalibrationReport = cameraReport;
        jsonSource.reliabilityPipelineReport = pipelineExecution.report;
        jsonSource.bootstrapResamples = 1;
        jsonSource.bootstrapConfidence = 0.95;
        const auto jsonWrite = handeye::writeJson(tempDir.filePath(QStringLiteral("samples.json")), jsonSource);
        const auto jsonRead = handeye::readJson(tempDir.filePath(QStringLiteral("samples.json")), &jsonDataset);
        handeye::CalibrationDataset pointJsonDataset;
        const auto pointJsonWrite = handeye::writeJson(tempDir.filePath(QStringLiteral("point_samples.json")), pointDataset);
        const auto pointJsonRead = handeye::readJson(tempDir.filePath(QStringLiteral("point_samples.json")), &pointJsonDataset);
        const auto yamlWrite = handeye::writeYaml(tempDir.filePath(QStringLiteral("result.yaml")), dataset);
        const auto txtWrite = results.isEmpty() ? handeye::IoResult{false, {}}
                                                 : handeye::writeResultTxt(tempDir.filePath(QStringLiteral("result.txt")), dataset, results.first());
        const auto cppWrite = results.isEmpty() ? handeye::IoResult{false, {}}
                                                 : handeye::writeResultCpp(tempDir.filePath(QStringLiteral("result.cpp")), dataset, results.first());
        const auto pythonWrite = results.isEmpty() ? handeye::IoResult{false, {}}
                                                   : handeye::writeResultPython(tempDir.filePath(QStringLiteral("result.py")), dataset, results.first());
        const bool ioOk = csvWrite.success && csvRead.success && jsonWrite.success && jsonRead.success
                          && yamlWrite.success && txtWrite.success && cppWrite.success && pythonWrite.success
                          && pointJsonWrite.success && pointJsonRead.success
                          && csvDataset.samples.size() == dataset.samples.size()
                          && jsonDataset.samples.size() == dataset.samples.size()
                          && pointJsonDataset.inputMode == handeye::CalibrationInputMode::FixedPoint3D
                          && pointJsonDataset.pointSamples.size() == pointDataset.pointSamples.size();
        const bool cameraJsonOk = jsonDataset.cameraIntrinsics.valid
                                  && matrix3MaxError(jsonDataset.cameraIntrinsics.cameraMatrix,
                                                     cameraReport.intrinsics.cameraMatrix) < 1e-12
                                  && jsonDataset.cameraIntrinsics.distortionCoeffs == cameraReport.intrinsics.distortionCoeffs
                                  && jsonDataset.cameraCalibrationReport.success;
        const bool pipelineJsonOk = jsonDataset.reliabilityPipelineReport.available
                                    && jsonDataset.reliabilityPipelineReport.bootstrapReport.success
                                    && jsonDataset.reliabilityPipelineReport.stages.size()
                                           == pipelineExecution.report.stages.size();

        handeye::MainWindow smokeWindow;
        const auto *tabs = smokeWindow.findChild<QTabWidget *>(QStringLiteral("mainTabs"));
        const QStringList expectedTabs = {QStringLiteral("首页"), QStringLiteral("采集"), QStringLiteral("参数"),
                                          QStringLiteral("相机内参"), QStringLiteral("手动输入"),
                                          QStringLiteral("当前数据"), QStringLiteral("标定结果")};
        bool uiTabsOk = tabs && tabs->count() == expectedTabs.size();
        if (uiTabsOk) {
            for (int index = 0; index < expectedTabs.size(); ++index)
                uiTabsOk = uiTabsOk && tabs->tabText(index) == expectedTabs.at(index);
        }
        const auto buttons = smokeWindow.findChildren<QPushButton *>();
        const auto hasButton = [&buttons](const QString &text) {
            return std::any_of(buttons.cbegin(), buttons.cend(), [&text](const QPushButton *button) {
                return button->text() == text;
            });
        };
        const bool uiActionsOk = hasButton(QStringLiteral("上传机器人坐标"))
                                 && hasButton(QStringLiteral("上传标定板图片"))
                                 && hasButton(QStringLiteral("开始相机标定"))
                                 && hasButton(QStringLiteral("应用到当前标定数据"))
                                 && hasButton(QStringLiteral("应用并计算五种算法"))
                                 && hasButton(QStringLiteral("五种算法自动比较并推荐"))
                                 && hasButton(QStringLiteral("执行完整可靠性流水线"));
        auto *cameraPage = smokeWindow.findChild<handeye::CameraCalibrationPage *>();
        if (cameraPage) {
            cameraPage->setBoardSpec(cameraBoard);
            cameraPage->setReport(cameraReport);
        }
        const auto *cameraTable = smokeWindow.findChild<QTableWidget *>(QStringLiteral("cameraCalibrationTable"));
        const bool uiCameraPageOk = cameraPage && cameraTable && cameraTable->rowCount() == cameraReport.samples.size();
        auto *manualPage = smokeWindow.findChild<handeye::ManualPosePage *>();
        const auto *manualTable = smokeWindow.findChild<QTableWidget *>(QStringLiteral("manualPoseTable"));
        const bool uiManualPageOk = manualPage && manualTable;
        auto *currentDataPage = smokeWindow.findChild<handeye::CurrentDataPage *>();
        if (currentDataPage) currentDataPage->setSamples(dataset.samples);
        const auto *currentDataTable = smokeWindow.findChild<QTableView *>(QStringLiteral("currentDataTable"));
        const bool uiDataOk = currentDataPage && currentDataTable && currentDataTable->model()
                              && currentDataTable->model()->rowCount() == dataset.samples.size();
        auto *resultPage = smokeWindow.findChild<handeye::CalibrationResultPage *>();
        if (resultPage) {
            resultPage->setResults(results);
            resultPage->showPipelineReport(pipelineExecution.report);
        }
        const auto *resultMatrix = smokeWindow.findChild<QPlainTextEdit *>(QStringLiteral("resultMatrix"));
        const bool uiResultOk = resultPage && resultMatrix
                                && resultMatrix->toPlainText().contains(QStringLiteral("camera→gripper"));
        const bool uiSmokeOk = uiTabsOk && uiActionsOk && uiCameraPageOk && uiManualPageOk
                               && uiDataOk && uiResultOk;

        const bool allOk = successCount == handeye::allMethods().size()
                           && recommendedCount == 1
                           && maxTruthError < 1e-5
                           && normalizationOk && independentValidationOk
                           && outlierDetectionOk && degeneracyOk && ioOk
                           && imagePoseOk && pairedImportOk && cameraCalibrationOk && cameraOutlierOk
                           && cameraTooFewOk && cameraResolutionOk && cameraJsonOk
                           && manualApplyOk && manualDuplicateRejected && manualQuaternionOk
                           && pointCalibrationOk && fixedTargetOk && qualityOk && nonlinearOk
                           && manualPointApplyOk && pipelineOk && pipelineJsonOk && uiSmokeOk;
        smokeStream << "success=" << successCount << "/" << handeye::allMethods().size() << Qt::endl;
        smokeStream << "recommended=" << recommendedCount << ",truth_max_error=" << maxTruthError << Qt::endl;
        smokeStream << "normalization=" << normalizationOk << ",independent_validation=" << independentValidationOk
                    << ",outlier_detection=" << outlierDetectionOk << ",degeneracy=" << degeneracyOk << Qt::endl;
        smokeStream << "io_roundtrip_and_exports=" << ioOk << Qt::endl;
        smokeStream << "image_pose_estimation=" << imagePoseOk
                    << ",paired_import=" << pairedImportOk << Qt::endl;
        smokeStream << "camera_calibration=" << cameraCalibrationOk
                    << ",camera_outlier=" << cameraOutlierOk
                    << ",camera_too_few=" << cameraTooFewOk
                    << ",camera_resolution=" << cameraResolutionOk
                    << ",camera_json=" << cameraJsonOk << Qt::endl;
        smokeStream << "manual_apply=" << manualApplyOk
                    << ",manual_duplicate_rejected=" << manualDuplicateRejected
                    << ",manual_quaternion=" << manualQuaternionOk << Qt::endl;
        smokeStream << "fixed_point_calibration=" << pointCalibrationOk
                    << ",fixed_target_pose=" << fixedTargetOk
                    << ",quality_score=" << qualityOk
                    << ",nonlinear_optimization=" << nonlinearOk
                    << ",manual_point_apply=" << manualPointApplyOk << Qt::endl;
        smokeStream << "reliability_pipeline=" << pipelineOk
                    << ",pipeline_json=" << pipelineJsonOk
                    << ",bootstrap=" << pipelineExecution.report.bootstrapReport.success
                    << ",removed=" << pipelineExecution.report.autoRemovedCount << Qt::endl;
        smokeStream << "ui_tabs=" << uiTabsOk << ",ui_actions=" << uiActionsOk
                    << ",ui_camera_page=" << uiCameraPageOk
                    << ",ui_manual_page=" << uiManualPageOk
                    << ",ui_data_table=" << uiDataOk << ",ui_result_matrix=" << uiResultOk << Qt::endl;
        qInfo() << "smoke-test" << allOk;
        return allOk ? 0 : 1;
    }
    handeye::MainWindow window;
    window.show();
    return app.exec();
}
