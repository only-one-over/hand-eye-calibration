#include "controllers/calibration_controller.h"
#include "core/board_pdf_generator.h"
#include "core/board_pdf_storage.h"
#include "core/calibration_service.h"
#include "core/matrix_utils.h"
#include "core/nonlinear_optimizer.h"
#include "core/normalized_huber.h"
#include "core/pose_conversion.h"
#include "core/pose_quality_service.h"
#include "core/reliability_pipeline_service.h"
#include "core/synthetic_dataset.h"
#include "io/dataset_io.h"
#include "io/pose_adapter.h"
#include "core/document_service.h"
#include "core/eye_to_hand_calibration_service.h"

#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <QtTest>

#include <algorithm>
#include <cmath>

using namespace handeye;

double matrixError(const cv::Matx44d &left, const cv::Matx44d &right)
{
    double error = 0.0;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            error = std::max(error, std::abs(left(row, col) - right(row, col)));
    return error;
}

cv::Vec3d pointOf(const cv::Matx44d &pose, const Vector3 &point)
{
    return {pose(0, 0) * point[0] + pose(0, 1) * point[1] + pose(0, 2) * point[2] + pose(0, 3),
            pose(1, 0) * point[0] + pose(1, 1) * point[1] + pose(1, 2) * point[2] + pose(1, 3),
            pose(2, 0) * point[0] + pose(2, 1) * point[1] + pose(2, 2) * point[2] + pose(2, 3)};
}

class HandeyeUnitTest : public QObject
{
    Q_OBJECT

private slots:
    void eulerAndRpyRoundTrip();
    void vendorAdaptersUseDistinctFieldMappings();
    void zeroQuaternionIsRejected();
    void normalizedHuberDownWeightsOutlier();
    void quaternionCsvFieldCountIsStrict();
    void parameterChangePreservesManualPose();
    void poseAdapterChangeRequiresReimport();
    void nonlinearReportsRemainSeparate();
    void bootstrapReportsSuccessRate();
    void bootstrapSmallSampleMode();
    void jsonRoundTripPersistsConventionAndMarkerSeparation();
    void boardPdfGenerationAndDocumentFallback();
    void eyeToHandPosePairsRecoverMatrices();
    void eyeToHandPointModeRecoversExtrinsics();
    void eyeToHandPointReportsRankCondition();
    void eyeToHandJsonRoundTripKeepsDirections();
    void eyeToHandReliabilityPipelineRuns();
};

void HandeyeUnitTest::eulerAndRpyRoundTrip()
{
    const Vector3 original{0.3, -0.2, 0.4};
    PoseInputSpec eulerSpec;
    eulerSpec.rotationFormat = RotationFormat::EulerXYZ;
    eulerSpec.convention = PoseConvention::EulerXYZIntrinsic;
    const auto euler = pose::normalize({original[0], original[1], original[2], 0.0}, {}, eulerSpec);
    QVERIFY(euler.success);
    const Vector4 eulerBack = pose::rotationToFormat(euler.rotation, RotationFormat::EulerXYZ,
                                                     AngleUnit::Radians);
    QVERIFY(std::abs(eulerBack[0] - original[0]) < 1e-9);
    QVERIFY(std::abs(eulerBack[1] - original[1]) < 1e-9);
    QVERIFY(std::abs(eulerBack[2] - original[2]) < 1e-9);

    PoseInputSpec rpySpec;
    rpySpec.rotationFormat = RotationFormat::RPY;
    rpySpec.convention = PoseConvention::RpyZyx;
    const auto rpy = pose::normalize({original[0], original[1], original[2], 0.0}, {}, rpySpec);
    QVERIFY(rpy.success);
    const Vector4 rpyBack = pose::rotationToFormat(rpy.rotation, RotationFormat::RPY,
                                                   AngleUnit::Radians);
    QVERIFY(std::abs(rpyBack[0] - original[0]) < 1e-9);
    QVERIFY(std::abs(rpyBack[1] - original[1]) < 1e-9);
    QVERIFY(std::abs(rpyBack[2] - original[2]) < 1e-9);
}

void HandeyeUnitTest::vendorAdaptersUseDistinctFieldMappings()
{
    const AdapterResult kuka = parseRobotPoseLine("0,0,0,90,0,0", PoseAdapterKind::Kuka, 1);
    const AdapterResult fanuc = parseRobotPoseLine("0,0,0,0,0,90", PoseAdapterKind::Fanuc, 1);
    QVERIFY(kuka.success);
    QVERIFY(fanuc.success);
    QVERIFY(matrixError(matrix::fromRodrigues(kuka.rotation, kuka.translation),
                        matrix::fromRodrigues(fanuc.rotation, fanuc.translation)) < 1e-9);
}

void HandeyeUnitTest::zeroQuaternionIsRejected()
{
    PoseInputSpec spec;
    spec.rotationFormat = RotationFormat::QuaternionWXYZ;
    const auto result = pose::normalize({0.0, 0.0, 0.0, 0.0}, {}, spec);
    QVERIFY(!result.success);
}

void HandeyeUnitTest::normalizedHuberDownWeightsOutlier()
{
    const auto normal = NormalizedHuber::evaluate({0.0, 0.0, 0.0}, {0.0001, 0.0, 0.0});
    const auto outlier = NormalizedHuber::evaluate({0.0, 0.0, 0.0}, {0.1, 0.0, 0.0});
    QCOMPARE(normal.weight, 1.0);
    QVERIFY(outlier.outlier);
    QVERIFY(outlier.weight < 1.0);
    QVERIFY(outlier.loss > normal.loss);
}

void HandeyeUnitTest::quaternionCsvFieldCountIsStrict()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("quaternion.csv"));
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Text));
    file.write("id,label,gripper_qw,gripper_qx,gripper_qy,gripper_qz,gripper_tx,gripper_ty,gripper_tz,target_qw,target_qx,target_qy,target_qz,target_tx,target_ty,target_tz,extra\n");
    file.write("1,a,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1\n");
    file.close();
    CalibrationDataset dataset;
    PoseInputSpec spec;
    spec.rotationFormat = RotationFormat::QuaternionWXYZ;
    QVERIFY(!readCsv(path, &dataset, spec).success);
}

void HandeyeUnitTest::parameterChangePreservesManualPose()
{
    const CalibrationDataset synthetic = makeSyntheticDataset();
    QVector<ManualPoseInput> inputs;
    for (const PoseSample &sample : synthetic.samples) {
        ManualPoseInput input;
        input.id = sample.id;
        input.tcpRotation = {sample.gripperRotation[0], sample.gripperRotation[1], sample.gripperRotation[2], 0.0};
        input.tcpTranslation = sample.gripperTranslation;
        input.cameraRotation = {sample.targetRotation[0], sample.targetRotation[1], sample.targetRotation[2], 0.0};
        input.cameraTranslation = sample.targetTranslation;
        inputs.append(input);
    }
    CalibrationController controller;
    QVERIFY(controller.applyManualPoseInputs(inputs, PoseInputSpec{}));
    const Vector3 targetBefore = controller.dataset().samples.first().targetTranslation;
    BoardSpec changed = controller.dataset().boardSpec;
    changed.innerCornersX += 1;
    controller.synchronizeParameters(controller.dataset().inputSpec, {}, {}, changed,
                                     controller.dataset().cameraIntrinsics,
                                     controller.dataset().passRotationRmseDeg,
                                     controller.dataset().passTranslationRmseM);
    QCOMPARE(controller.dataset().samples.first().imageStatus, ImageSampleStatus::ManualPose);
    QCOMPARE(controller.dataset().samples.first().targetTranslation, targetBefore);
}

void HandeyeUnitTest::poseAdapterChangeRequiresReimport()
{
    const CalibrationDataset synthetic = makeSyntheticDataset();
    QVector<ManualPoseInput> inputs;
    for (const PoseSample &sample : synthetic.samples) {
        ManualPoseInput input;
        input.id = sample.id;
        input.tcpRotation = {sample.gripperRotation[0], sample.gripperRotation[1], sample.gripperRotation[2], 0.0};
        input.tcpTranslation = sample.gripperTranslation;
        input.cameraRotation = {sample.targetRotation[0], sample.targetRotation[1], sample.targetRotation[2], 0.0};
        input.cameraTranslation = sample.targetTranslation;
        inputs.append(input);
    }
    CalibrationController controller;
    QVERIFY(controller.applyManualPoseInputs(inputs, PoseInputSpec{}));
    QVERIFY(!controller.dataset().poseDataNeedsReimport);

    PoseInputSpec changed = controller.dataset().inputSpec;
    changed.adapter = PoseAdapterKind::Kuka;
    changed.convention = PoseConvention::KukaAbcZyx;
    QSignalSpy reimportSpy(&controller, &CalibrationController::poseDataReimportRequired);
    controller.synchronizeParameters(changed, controller.dataset().robotName,
                                     controller.dataset().cameraName, controller.dataset().boardSpec,
                                     controller.dataset().cameraIntrinsics,
                                     controller.dataset().passRotationRmseDeg,
                                     controller.dataset().passTranslationRmseM,
                                     controller.dataset().mode, controller.dataset().inputMode);
    QVERIFY(controller.dataset().poseDataNeedsReimport);
    QCOMPARE(reimportSpy.count(), 1);
    QSignalSpy errorSpy(&controller, &CalibrationController::error);
    controller.calculateSelected(CalibrationMethod::Tsai);
    QVERIFY(errorSpy.count() > 0);
    QVERIFY(controller.applyManualPoseInputs(inputs, changed));
    QVERIFY(!controller.dataset().poseDataNeedsReimport);

    CalibrationController namedController;
    QVERIFY(namedController.applyManualPoseInputs(inputs, PoseInputSpec{}));
    namedController.synchronizeParameters(namedController.dataset().inputSpec, QStringLiteral("Robot-A"),
                                          namedController.dataset().cameraName,
                                          namedController.dataset().boardSpec,
                                          namedController.dataset().cameraIntrinsics,
                                          namedController.dataset().passRotationRmseDeg,
                                          namedController.dataset().passTranslationRmseM,
                                          namedController.dataset().mode, namedController.dataset().inputMode);
    QVERIFY(!namedController.dataset().poseDataNeedsReimport);
}

void HandeyeUnitTest::nonlinearReportsRemainSeparate()
{
    CalibrationDataset dataset = makeSyntheticDataset();
    dataset.samples[0].targetTranslation[0] += 0.012;
    dataset.samples[0].targetRotation[1] += 0.035;
    const CalibrationResult seed = CalibrationService::calibrate(dataset, CalibrationMethod::Tsai);
    QVERIFY(seed.success);
    const CalibrationResult refined = NonlinearOptimizer::refinePose(dataset, seed);
    QVERIFY(refined.success);

    const AxXbReport independentAxXb = CalibrationService::evaluateAxXb(dataset, refined.cameraToGripper);
    const FixedTargetPoseReport independentFixed = PoseQualityService::computeFixedTargetPose(
        dataset, refined.cameraToGripper);
    QVERIFY(std::abs(refined.axXbReport.rotationRmseDeg - independentAxXb.rotationRmseDeg) < 1e-12);
    QVERIFY(std::abs(refined.axXbReport.translationRmseM - independentAxXb.translationRmseM) < 1e-12);
    QVERIFY(std::abs(refined.fixedTargetReport.rotationRmseDeg - independentFixed.rotationRmseDeg) < 1e-12);
    QVERIFY(std::abs(refined.fixedTargetReport.translationRmseM - independentFixed.translationRmseM) < 1e-12);
    QCOMPARE(refined.axXbReport.sampleCount, independentAxXb.sampleCount);
    QCOMPARE(refined.fixedTargetReport.samples.size(), independentFixed.samples.size());
    QVERIFY(std::abs(refined.axXbReport.translationRmseM - refined.fixedTargetReport.translationRmseM) > 1e-9);

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    CalibrationDataset serialized;
    serialized.samples = dataset.samples;
    serialized.results = {refined};
    const QString path = directory.filePath(QStringLiteral("reports.json"));
    QVERIFY(writeJson(path, serialized).success);
    CalibrationDataset restored;
    QVERIFY(readJson(path, &restored).success);
    QVERIFY(std::abs(restored.results.first().axXbReport.translationRmseM
                     - refined.axXbReport.translationRmseM) < 1e-12);
    QVERIFY(std::abs(restored.results.first().fixedTargetReport.translationRmseM
                     - refined.fixedTargetReport.translationRmseM) < 1e-12);
}

void HandeyeUnitTest::bootstrapReportsSuccessRate()
{
    CalibrationDataset dataset = makeSyntheticDataset();
    const ReliabilityPipelineExecution execution = ReliabilityPipelineService::run(dataset, 1, 0.95);
    QVERIFY(execution.report.bootstrapReport.available);
    QCOMPARE(execution.report.bootstrapReport.successRate,
             execution.report.bootstrapReport.requestedResamples > 0
                 ? static_cast<double>(execution.report.bootstrapReport.successfulResamples)
                       / execution.report.bootstrapReport.requestedResamples
                 : 0.0);
}

void HandeyeUnitTest::bootstrapSmallSampleMode()
{
    for (int count : {5, 7}) {
        const CalibrationDataset dataset = makeSyntheticDataset(count);
        const ReliabilityPipelineExecution execution = ReliabilityPipelineService::run(dataset, 2, 0.95);
        const BootstrapReport &report = execution.report.bootstrapReport;
        QVERIFY2(report.available, qPrintable(QStringLiteral("count=%1: %2").arg(count).arg(execution.report.message)));
        QVERIFY(report.smallSampleMode);
        QVERIFY(!report.uncertaintyReliable);
        QCOMPARE(report.minimumUniqueSamples, 3);
        QCOMPARE(report.successRate,
                 report.requestedResamples > 0
                     ? static_cast<double>(report.successfulResamples) / report.requestedResamples : 0.0);
    }
}

void HandeyeUnitTest::jsonRoundTripPersistsConventionAndMarkerSeparation()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());

    CalibrationDataset source = makeSyntheticDataset();
    source.inputSpec.adapter = PoseAdapterKind::Kuka;
    source.inputSpec.convention = PoseConvention::KukaAbcZyx;
    source.boardSpec.markerSeparationM = 0.012;
    source.poseDataNeedsReimport = true;
    source.reliabilityPipelineReport.bootstrapReport.available = true;
    source.reliabilityPipelineReport.bootstrapReport.smallSampleMode = true;
    source.reliabilityPipelineReport.bootstrapReport.uncertaintyReliable = false;
    source.reliabilityPipelineReport.bootstrapReport.successRate = 0.75;
    const QString path = directory.filePath(QStringLiteral("roundtrip.json"));
    QVERIFY(writeJson(path, source).success);

    CalibrationDataset restored;
    QVERIFY(readJson(path, &restored).success);
    QCOMPARE(restored.inputSpec.convention, PoseConvention::KukaAbcZyx);
    QCOMPARE(restored.boardSpec.markerSeparationM, 0.012);
    QVERIFY(restored.poseDataNeedsReimport);
    QVERIFY(restored.reliabilityPipelineReport.bootstrapReport.smallSampleMode);
    QVERIFY(!restored.reliabilityPipelineReport.bootstrapReport.uncertaintyReliable);
    QCOMPARE(restored.reliabilityPipelineReport.bootstrapReport.successRate, 0.75);
}

void HandeyeUnitTest::boardPdfGenerationAndDocumentFallback()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());

    BoardSpec chessboard;
    const BoardPdfReport chessReport = BoardPdfGenerator::generate(
        chessboard, directory.filePath(QStringLiteral("chessboard.pdf")), BoardPdfOutputMode::CustomSize);
    QVERIFY(chessReport.success);
    QCOMPARE(chessReport.pageCount, 1);
    QCOMPARE(chessReport.widthMm, 250.0);
    QCOMPARE(chessReport.heightMm, 175.0);
    QVERIFY(QFileInfo(chessReport.outputPath).size() > 100);
    const QString defaultDirectory = BoardPdfStorage::directory();
    QVERIFY(!defaultDirectory.isEmpty());
    QCOMPARE(QFileInfo(defaultDirectory).fileName(), QStringLiteral("board_pdfs"));
    QCOMPARE(QFileInfo(QFileInfo(defaultDirectory).absolutePath()).fileName(),
             QStringLiteral("HandEyeCalibration"));
    const QString chessName = BoardPdfStorage::suggestedFileName(
        chessboard, BoardPdfOutputMode::CustomSize);
    QVERIFY(chessName.startsWith(QStringLiteral("chessboard_9x6_25p000mm_single_")));
    QVERIFY(chessName.endsWith(QStringLiteral(".pdf")));

    BoardSpec charuco = chessboard;
    charuco.pattern = BoardPattern::Charuco;
    const BoardPdfReport charucoReport = BoardPdfGenerator::generate(
        charuco, directory.filePath(QStringLiteral("charuco.pdf")), BoardPdfOutputMode::CustomSize);
    QVERIFY(charucoReport.success);
    QVERIFY(BoardPdfStorage::suggestedFileName(charuco, BoardPdfOutputMode::CustomSize)
                .contains(QStringLiteral("charuco_9x6_25p000mm_marker18p750mm_dict0_single_")));

    BoardSpec grid = chessboard;
    grid.pattern = BoardPattern::ArucoGrid;
    const BoardPdfReport gridReport = BoardPdfGenerator::generate(
        grid, directory.filePath(QStringLiteral("grid.pdf")), BoardPdfOutputMode::A4Tiled);
    QVERIFY(gridReport.success);
    QVERIFY(gridReport.pageCount >= 1);
    QVERIFY(BoardPdfStorage::suggestedFileName(grid, BoardPdfOutputMode::A4Tiled)
                .contains(QStringLiteral("aruco_grid_5x7_marker18p750mm_gap5p000mm_dict0_a4_")));

    const BoardPdfReport duplicateReport = BoardPdfGenerator::generate(
        chessboard, chessReport.outputPath, BoardPdfOutputMode::CustomSize);
    QVERIFY(!duplicateReport.success);
    const BoardPdfReport reusedReport = BoardPdfGenerator::describeExisting(
        chessboard, chessReport.outputPath, BoardPdfOutputMode::CustomSize);
    QVERIFY(reusedReport.success);
    QVERIFY(reusedReport.reused);
    QCOMPARE(reusedReport.outputPath, chessReport.outputPath);
    QCOMPARE(reusedReport.pageCount, chessReport.pageCount);

    const QVector<DocumentInfo> documents = DocumentService::listDocuments();
    QCOMPARE(documents.size(), 3);
    for (const DocumentInfo &document : documents) {
        QVERIFY(document.available);
        QVERIFY(!document.external);
        QVERIFY(document.source.contains(QStringLiteral("内置")));
    }

    CalibrationDataset dataset;
    dataset.lastBoardPdfReport = reusedReport;
    const QString jsonPath = directory.filePath(QStringLiteral("board_report.json"));
    QVERIFY(writeJson(jsonPath, dataset).success);
    CalibrationDataset restored;
    QVERIFY(readJson(jsonPath, &restored).success);
    QCOMPARE(restored.lastBoardPdfReport.outputPath, reusedReport.outputPath);
    QCOMPARE(restored.lastBoardPdfReport.widthMm, reusedReport.widthMm);
    QCOMPARE(restored.lastBoardPdfReport.outputMode, reusedReport.outputMode);
    QVERIFY(restored.lastBoardPdfReport.reused);
}

void HandeyeUnitTest::eyeToHandPosePairsRecoverMatrices()
{
    CalibrationDataset dataset;
    dataset.mode = CalibrationMode::EyeToHand;
    dataset.inputMode = CalibrationInputMode::PosePairs;
    const cv::Matx44d cameraToBase = matrix::fromRodrigues({0.2, -0.15, 0.1}, {0.4, -0.2, 1.0});
    const cv::Matx44d targetToGripper = matrix::fromRodrigues({-0.1, 0.12, 0.05}, {0.08, -0.03, 0.2});
    for (int index = 0; index < 10; ++index) {
        const Vector3 rotation{0.08 * std::sin(index * 0.7), 0.13 * std::cos(index * 0.4),
                               0.11 * std::sin(index * 0.35 + 0.2)};
        const Vector3 translation{0.05 * index, -0.02 * index + 0.01 * std::sin(index),
                                 0.25 + 0.03 * index};
        const cv::Matx44d robot = matrix::fromRodrigues(rotation, translation);
        const cv::Matx44d target = matrix::inverse(cameraToBase) * robot * targetToGripper;
        PoseSample sample;
        sample.id = index + 1;
        sample.gripperRotation = rotation;
        sample.gripperTranslation = translation;
        sample.targetRotation = matrix::toRodrigues(target.get_minor<3, 3>(0, 0));
        sample.targetTranslation = {target(0, 3), target(1, 3), target(2, 3)};
        dataset.samples.append(sample);
    }
    const CalibrationResult result = EyeToHandCalibrationService::calibrate(
        dataset, CalibrationMethod::RobotWorldShah);
    QVERIFY2(result.success, qPrintable(result.message));
    QVERIFY(matrixError(matrix::toMat(result.cameraToBase), cameraToBase) < 1e-6);
    QVERIFY(matrixError(matrix::toMat(result.targetToGripper), targetToGripper) < 1e-6);
    QVERIFY(result.eyeToHandPoseReport.success);
    QVERIFY(result.eyeToHandPoseReport.translationRmseM < 1e-8);
}

void HandeyeUnitTest::eyeToHandPointModeRecoversExtrinsics()
{
    CalibrationDataset dataset;
    dataset.mode = CalibrationMode::EyeToHand;
    dataset.inputMode = CalibrationInputMode::FixedPoint3D;
    const cv::Matx44d cameraToBase = matrix::fromRodrigues({0.18, -0.1, 0.12}, {0.3, 0.15, 0.9});
    const cv::Vec3d pointInGripper(0.04, -0.03, 0.12);
    for (int index = 0; index < 12; ++index) {
        const Vector3 rotation{0.11 * std::sin(index * 0.5), 0.16 * std::cos(index * 0.33),
                               0.09 * std::sin(index * 0.27 + 0.1)};
        const Vector3 translation{0.03 * index, 0.02 * std::sin(index * 0.8), 0.2 + 0.04 * index};
        const cv::Matx44d robot = matrix::fromRodrigues(rotation, translation);
        const cv::Vec3d pointCamera = pointOf(matrix::inverse(cameraToBase) * robot,
                                              {pointInGripper[0], pointInGripper[1], pointInGripper[2]});
        PointSample sample;
        sample.id = index + 1;
        sample.gripperRotation = rotation;
        sample.gripperTranslation = translation;
        sample.cameraPoint = {pointCamera[0], pointCamera[1], pointCamera[2]};
        dataset.pointSamples.append(sample);
    }
    const CalibrationResult result = EyeToHandCalibrationService::calibrate(
        dataset, CalibrationMethod::PointBased);
    QVERIFY2(result.success, qPrintable(result.message));
    QVERIFY(matrixError(matrix::toMat(result.cameraToBase), cameraToBase) < 1e-4);
    QVERIFY(std::abs(result.pointInGripper[0] - pointInGripper[0]) < 1e-4);
    QVERIFY(std::abs(result.pointInGripper[1] - pointInGripper[1]) < 1e-4);
    QVERIFY(std::abs(result.pointInGripper[2] - pointInGripper[2]) < 1e-4);
    QVERIFY(result.eyeToHandPointReport.rmseM < 1e-6);
}

void HandeyeUnitTest::eyeToHandPointReportsRankCondition()
{
    CalibrationDataset dataset;
    dataset.mode = CalibrationMode::EyeToHand;
    dataset.inputMode = CalibrationInputMode::FixedPoint3D;
    const cv::Matx44d cameraToBase = matrix::fromRodrigues({0.18, -0.1, 0.12}, {0.3, 0.15, 0.9});
    const cv::Vec3d pointInGripper(0.04, -0.03, 0.12);
    for (int index = 0; index < 12; ++index) {
        const Vector3 rotation{0.11 * std::sin(index * 0.5), 0.16 * std::cos(index * 0.33),
                               0.09 * std::sin(index * 0.27 + 0.1)};
        const Vector3 translation{0.03 * index, 0.02 * std::sin(index * 0.8), 0.2 + 0.04 * index};
        const cv::Matx44d robot = matrix::fromRodrigues(rotation, translation);
        const cv::Vec3d pointCamera = pointOf(matrix::inverse(cameraToBase) * robot,
                                              {pointInGripper[0], pointInGripper[1], pointInGripper[2]});
        PointSample sample;
        sample.id = index + 1;
        sample.gripperRotation = rotation;
        sample.gripperTranslation = translation;
        sample.cameraPoint = {pointCamera[0], pointCamera[1], pointCamera[2]};
        dataset.pointSamples.append(sample);
    }
    const CalibrationResult result = EyeToHandCalibrationService::calibrate(dataset, CalibrationMethod::PointBased);
    QVERIFY(result.eyeToHandPointReport.available);
    QCOMPARE(result.eyeToHandPointReport.linearRank, 15);
    QVERIFY(result.eyeToHandPointReport.fullRank);
    QVERIFY(std::isfinite(result.eyeToHandPointReport.linearConditionNumber));

    CalibrationDataset degenerate = dataset;
    for (PointSample &sample : degenerate.pointSamples) {
        sample.gripperRotation = {};
        sample.gripperTranslation = {};
        sample.cameraPoint = {0.1, 0.2, 0.5};
    }
    const CalibrationResult degenerateResult = EyeToHandCalibrationService::calibrate(
        degenerate, CalibrationMethod::PointBased);
    QVERIFY(!degenerateResult.success);
    QVERIFY(degenerateResult.eyeToHandPointReport.linearRank < 15
            || !degenerateResult.eyeToHandPointReport.conditionAcceptable);
}

void HandeyeUnitTest::eyeToHandJsonRoundTripKeepsDirections()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    CalibrationDataset source;
    source.mode = CalibrationMode::EyeToHand;
    source.inputMode = CalibrationInputMode::PosePairs;
    CalibrationResult result;
    result.method = CalibrationMethod::RobotWorldShah;
    result.success = true;
    result.cameraToBase = matrix::toArray(matrix::fromRodrigues({0.1, 0.2, -0.1}, {1.0, 2.0, 3.0}));
    result.targetToGripper = matrix::toArray(matrix::fromRodrigues({-0.1, 0.1, 0.05}, {0.1, 0.2, 0.3}));
    result.eyeToHandPoseReport.available = true;
    result.eyeToHandPoseReport.success = true;
    result.eyeToHandPoseReport.cameraToBase = result.cameraToBase;
    result.eyeToHandPoseReport.targetToGripper = result.targetToGripper;
    source.results.append(result);
    const QString path = directory.filePath(QStringLiteral("eye_to_hand.json"));
    QVERIFY(writeJson(path, source).success);
    CalibrationDataset restored;
    QVERIFY(readJson(path, &restored).success);
    QCOMPARE(restored.mode, CalibrationMode::EyeToHand);
    QCOMPARE(restored.results.size(), 1);
    QCOMPARE(restored.results.first().method, CalibrationMethod::RobotWorldShah);
    QVERIFY(matrixError(matrix::toMat(restored.results.first().cameraToBase),
                        matrix::toMat(result.cameraToBase)) < 1e-12);
    QVERIFY(restored.results.first().eyeToHandPoseReport.available);
}

void HandeyeUnitTest::eyeToHandReliabilityPipelineRuns()
{
    CalibrationDataset dataset;
    dataset.mode = CalibrationMode::EyeToHand;
    dataset.inputMode = CalibrationInputMode::PosePairs;
    const cv::Matx44d cameraToBase = matrix::fromRodrigues({0.15, -0.11, 0.08}, {0.3, 0.1, 0.8});
    const cv::Matx44d targetToGripper = matrix::fromRodrigues({-0.08, 0.1, 0.04}, {0.06, 0.02, 0.16});
    for (int index = 0; index < 10; ++index) {
        const Vector3 rotation{0.1 * std::sin(index * 0.6), 0.12 * std::cos(index * 0.31),
                               0.08 * std::sin(index * 0.45)};
        const Vector3 translation{0.04 * index, 0.02 * std::cos(index * 0.4), 0.25 + 0.03 * index};
        const cv::Matx44d robot = matrix::fromRodrigues(rotation, translation);
        const cv::Matx44d target = matrix::inverse(cameraToBase) * robot * targetToGripper;
        PoseSample sample;
        sample.id = index + 1;
        sample.gripperRotation = rotation;
        sample.gripperTranslation = translation;
        sample.targetRotation = matrix::toRodrigues(target.get_minor<3, 3>(0, 0));
        sample.targetTranslation = {target(0, 3), target(1, 3), target(2, 3)};
        dataset.samples.append(sample);
    }
    const ReliabilityPipelineExecution execution = ReliabilityPipelineService::run(dataset, 2, 0.95);
    QVERIFY2(execution.report.success, qPrintable(execution.report.message));
    QVERIFY(execution.finalResult.eyeToHandPoseReport.available);
    QVERIFY(execution.report.bootstrapReport.available);
    QVERIFY(execution.report.bootstrapReport.successRate >= 0.0
            && execution.report.bootstrapReport.successRate <= 1.0);
}

QTEST_MAIN(HandeyeUnitTest)

#include "handeye_unit_test.moc"
