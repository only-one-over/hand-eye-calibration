#include "controllers/calibration_controller.h"
#include "core/matrix_utils.h"
#include "core/normalized_huber.h"
#include "core/pose_conversion.h"
#include "core/reliability_pipeline_service.h"
#include "core/synthetic_dataset.h"
#include "io/dataset_io.h"
#include "io/pose_adapter.h"

#include <QFile>
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
    void bootstrapReportsSuccessRate();
    void jsonRoundTripPersistsConventionAndMarkerSeparation();
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

void HandeyeUnitTest::jsonRoundTripPersistsConventionAndMarkerSeparation()
{
    QTemporaryDir directory;
    QVERIFY(directory.isValid());

    CalibrationDataset source = makeSyntheticDataset();
    source.inputSpec.adapter = PoseAdapterKind::Kuka;
    source.inputSpec.convention = PoseConvention::KukaAbcZyx;
    source.boardSpec.markerSeparationM = 0.012;
    const QString path = directory.filePath(QStringLiteral("roundtrip.json"));
    QVERIFY(writeJson(path, source).success);

    CalibrationDataset restored;
    QVERIFY(readJson(path, &restored).success);
    QCOMPARE(restored.inputSpec.convention, PoseConvention::KukaAbcZyx);
    QCOMPARE(restored.boardSpec.markerSeparationM, 0.012);
}

QTEST_GUILESS_MAIN(HandeyeUnitTest)

#include "handeye_unit_test.moc"
