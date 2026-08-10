#pragma once

#include "core/board_pdf_generator.h"
#include "domain/calibration_types.h"

#include <QMainWindow>
#include <QVector>

class QTabWidget;

namespace handeye {

class CalibrationController;
class HomePage;
class CapturePage;
class ParametersPage;
class BoardPdfPage;
class CameraCalibrationPage;
class ManualPosePage;
class CurrentDataPage;
class CalibrationResultPage;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void onStartCalibration();
    void onParametersNext();
    void onCaptureNext();
    void onDataNext();
    void onNewDataset();
    void onGenerateDemo();
    void onImportRobotPoseCsv();
    void onImportCalibrationImages();
    void onSelectCameraCalibrationImages();
    void onDetectCameraCalibrationImages();
    void onCalibrateCameraIntrinsics();
    void onApplyCameraIntrinsics();
    void onClearCameraCalibrationImages();
    void onApplyManualPointInputs(const QVector<PointSample> &samples,
                                  const PoseInputSpec &spec, bool calculateAll);
    void onApplyManualPoseInputs(const QVector<ManualPoseInput> &samples,
                                 const PoseInputSpec &spec, bool calculateAll);
    void onImportPoseImageCsv();
    void onImportProcessedCsv();
    void onImportValidationCsv();
    void onImportJson();
    void onExportProcessedCsv();
    void onExportRequested(const QString &kind);
    void onCalculateSelected();
    void onCalculateAll();
    void onRunReliabilityPipeline(int bootstrapResamples, double confidenceLevel);
    void onComputeFixedTarget(int referenceSampleId);
    void onOptimizeRecommended();
    void onProcessBoardImages();
    void onGenerateBoardPdf(BoardPdfOutputMode mode);
    void onSaveAsBoardPdf(BoardPdfOutputMode mode);
    void onOpenBoardPdf(const QString &path);
    void onOpenBoardPdfDirectory();
    void onOpenDocumentsDirectory();
    void onOpenDocument(const QString &fileName);
    void onSamplesChanged(const QVector<PoseSample> &samples);
    void onResultsChanged(const QVector<CalibrationResult> &results);
    void onReliabilityChanged(const CalibrationResult &result);
    void onMatrixChanged(const CalibrationResult &result);
    void onStatusChanged(const QString &text);
    void onLogMessage(const QString &message);
    void onInputSpecChanged(const QString &robot, const QString &camera);
    void onImageProcessingFinished(int processed, int succeeded);
    void onCameraCalibrationChanged(const CameraCalibrationReport &report);
    void onCameraCalibrationStarted();
    void onCameraCalibrationFinished();
    void onCalculationStarted();
    void onCalculationFinished();
    void onReliabilityPipelineStarted();
    void onReliabilityPipelineFinished();
    void onReliabilityPipelineChanged(const ReliabilityPipelineReport &report);
    void onError(const QString &title, const QString &message);

private:
    enum Page { HomePageIndex = 0, CapturePageIndex = 1, ParametersPageIndex = 2,
                CameraCalibrationPageIndex = 3, ManualPosePageIndex = 4,
                BoardPdfPageIndex = 5, CurrentDataPageIndex = 6, ResultPageIndex = 7 };

    void buildPages();
    void buildMenuBar();
    void connectSignals();
    void syncParametersToController();
    void navigateToPage(int index);
    void updateBatchSummary(const QVector<PoseSample> &samples);
    bool confirmPairing(const QString &operation);
    CalibrationResult selectedResult() const;

    QTabWidget *m_tabs = nullptr;
    HomePage *m_homePage = nullptr;
    CapturePage *m_capturePage = nullptr;
    ParametersPage *m_parametersPage = nullptr;
    BoardPdfPage *m_boardPdfPage = nullptr;
    CameraCalibrationPage *m_cameraCalibrationPage = nullptr;
    ManualPosePage *m_manualPosePage = nullptr;
    CurrentDataPage *m_currentDataPage = nullptr;
    CalibrationResultPage *m_resultPage = nullptr;
    CalibrationController *m_controller = nullptr;
};

} // namespace handeye
