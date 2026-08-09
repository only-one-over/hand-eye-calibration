#pragma once

#include "domain/calibration_types.h"

#include <QObject>
#include <QString>
#include <QVector>

namespace handeye {

class CalibrationController : public QObject
{
    Q_OBJECT

public:
    explicit CalibrationController(QObject *parent = nullptr);

    const CalibrationDataset &dataset() const;

    // Data operations
    void newDataset();
    void generateDemo();
    void importCsv(const QString &path);
    void importRobotPoseCsv(const QString &path);
    void importCalibrationImages(const QStringList &paths);
    void importCameraCalibrationImages(const QStringList &paths);
    void detectCameraCalibrationImages();
    void calibrateCameraIntrinsics();
    void applyCameraIntrinsics();
    void clearCameraCalibrationImages();
    void importPoseImageCsv(const QString &path);
    void importValidationCsv(const QString &path);
    void importJson(const QString &path);
    void exportCsv(const QString &path);
    void exportJson(const QString &path);
    void exportYaml(const QString &path);
    void exportTxt(const QString &path, const CalibrationResult &result);
    void exportCpp(const QString &path, const CalibrationResult &result);
    void exportPython(const QString &path, const CalibrationResult &result);
    void deleteSamples(const QVector<int> &ids);
    bool applyManualPoseInputs(const QVector<ManualPoseInput> &inputs,
                               const PoseInputSpec &spec);
    bool applyManualPointInputs(const QVector<PointSample> &samples,
                                const PoseInputSpec &spec);

    FixedTargetPoseReport computeFixedTargetPose(const CalibrationResult &result,
                                                 int referenceSampleId = -1);
    CalibrationResult optimizeRecommendedResult();
    PoseQualityReport evaluatePoseQuality() const;

    // Calculation (async)
    void calculateSelected(CalibrationMethod method);
    void calculateAll();
    void runReliabilityPipeline(int bootstrapResamples = -1,
                                double confidenceLevel = -1.0);

    // Input spec update
    void updateInputSpec(const PoseInputSpec &spec, const QString &robot, const QString &camera);
    void updateImageProcessing(const BoardSpec &board, const CameraIntrinsics &intrinsics);
    void updateReliabilityThresholds(double rotationRmseDeg, double translationRmseM);
    bool processBoardImages();
    bool ensureTargetPosesReady();

signals:
    void samplesChanged(const QVector<PoseSample> &samples);
    void resultsChanged(const QVector<CalibrationResult> &results);
    void reliabilityChanged(const CalibrationResult &result);
    void matrixChanged(const CalibrationResult &result);
    void statusChanged(const QString &text);
    void logMessage(const QString &message);
    void inputSpecChanged(const QString &robot, const QString &camera);
    void calculationStarted();
    void calculationFinished();
    void reliabilityPipelineStarted();
    void reliabilityPipelineFinished();
    void reliabilityPipelineChanged(const ReliabilityPipelineReport &report);
    void imageProcessingFinished(int processed, int succeeded);
    void cameraCalibrationChanged(const CameraCalibrationReport &report);
    void cameraCalibrationStarted();
    void cameraCalibrationFinished();
    void error(const QString &title, const QString &message);

private:
    void applyResiduals(const ReliabilityReport &report);
    void emitDatasetChanged();
    CalibrationResult recommendedResult() const;
    void emitCameraCalibrationChanged();

    CalibrationDataset m_dataset;
};

} // namespace handeye
