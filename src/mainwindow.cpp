#include "mainwindow.h"

#include "controllers/calibration_controller.h"
#include "views/camera_calibration_page.h"
#include "views/calibration_result_page.h"
#include "views/capture_page.h"
#include "views/current_data_page.h"
#include "views/home_page.h"
#include "views/manual_pose_page.h"
#include "views/parameters_page.h"

#include <QAction>
#include <QApplication>
#include <QDateTime>
#include <QFileDialog>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>
#include <QTabWidget>

#include <algorithm>

namespace handeye {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle(QStringLiteral("手眼标定工具"));
    resize(1480, 920);
    m_controller = new CalibrationController(this);
    buildPages();
    buildMenuBar();
    connectSignals();
}

void MainWindow::buildPages()
{
    m_tabs = new QTabWidget(this);
    m_tabs->setObjectName(QStringLiteral("mainTabs"));
    m_homePage = new HomePage(m_tabs);
    m_capturePage = new CapturePage(m_tabs);
    m_parametersPage = new ParametersPage(m_tabs);
    m_cameraCalibrationPage = new CameraCalibrationPage(m_tabs);
    m_manualPosePage = new ManualPosePage(m_tabs);
    m_currentDataPage = new CurrentDataPage(m_tabs);
    m_resultPage = new CalibrationResultPage(m_tabs);
    m_tabs->addTab(m_homePage, QStringLiteral("首页"));
    m_tabs->addTab(m_capturePage, QStringLiteral("采集"));
    m_tabs->addTab(m_parametersPage, QStringLiteral("参数"));
    m_tabs->addTab(m_cameraCalibrationPage, QStringLiteral("相机内参"));
    m_tabs->addTab(m_manualPosePage, QStringLiteral("手动输入"));
    m_tabs->addTab(m_currentDataPage, QStringLiteral("当前数据"));
    m_tabs->addTab(m_resultPage, QStringLiteral("标定结果"));
    setCentralWidget(m_tabs);
}

void MainWindow::buildMenuBar()
{
    auto *fileMenu = menuBar()->addMenu(QStringLiteral("文件"));
    fileMenu->addAction(QStringLiteral("新建当前批次"), this, &MainWindow::onNewDataset);
    fileMenu->addAction(QStringLiteral("上传机器人坐标"), this, &MainWindow::onImportRobotPoseCsv);
    fileMenu->addAction(QStringLiteral("上传标定板图片"), this, &MainWindow::onImportCalibrationImages);
    fileMenu->addAction(QStringLiteral("导入配对 CSV（高级）"), this, &MainWindow::onImportPoseImageCsv);
    fileMenu->addAction(QStringLiteral("导入已处理位姿 CSV"), this, &MainWindow::onImportProcessedCsv);
    fileMenu->addAction(QStringLiteral("导入独立验证 CSV"), this, &MainWindow::onImportValidationCsv);
    fileMenu->addAction(QStringLiteral("导入 JSON"), this, &MainWindow::onImportJson);
    fileMenu->addAction(QStringLiteral("导出标准 CSV"), this, &MainWindow::onExportProcessedCsv);
    fileMenu->addSeparator();
    fileMenu->addAction(QStringLiteral("生成合成示例数据"), this, &MainWindow::onGenerateDemo);

    auto *viewMenu = menuBar()->addMenu(QStringLiteral("查看"));
    viewMenu->addAction(QStringLiteral("首页"), this, [this] { navigateToPage(HomePageIndex); });
    viewMenu->addAction(QStringLiteral("相机内参"), this, [this] { navigateToPage(CameraCalibrationPageIndex); });
    viewMenu->addAction(QStringLiteral("手动输入"), this, [this] { navigateToPage(ManualPosePageIndex); });
    viewMenu->addAction(QStringLiteral("当前数据"), this, [this] { navigateToPage(CurrentDataPageIndex); });
    viewMenu->addAction(QStringLiteral("标定结果"), this, [this] { navigateToPage(ResultPageIndex); });

    auto *exportMenu = menuBar()->addMenu(QStringLiteral("导出"));
    for (const QString &kind : {QStringLiteral("json"), QStringLiteral("yaml"), QStringLiteral("txt"),
                                QStringLiteral("cpp"), QStringLiteral("python")})
        exportMenu->addAction(QStringLiteral("导出 ") + kind.toUpper(), this,
                              [this, kind] { onExportRequested(kind); });
}

void MainWindow::connectSignals()
{
    connect(m_tabs, &QTabWidget::currentChanged, this, [this](int index) {
        if (index == ParametersPageIndex) syncParametersToController();
    });
    connect(m_homePage, &HomePage::navigateRequested, this, &MainWindow::navigateToPage);
    connect(m_capturePage, &CapturePage::uploadRobotRequested, this, &MainWindow::onImportRobotPoseCsv);
    connect(m_capturePage, &CapturePage::uploadImagesRequested, this, &MainWindow::onImportCalibrationImages);
    connect(m_capturePage, &CapturePage::processRequested, this, &MainWindow::onProcessBoardImages);
    connect(m_capturePage, &CapturePage::viewDataRequested, this, [this] { navigateToPage(CurrentDataPageIndex); });
    connect(m_capturePage, &CapturePage::viewResultsRequested, this, [this] { navigateToPage(ResultPageIndex); });
    connect(m_parametersPage, &ParametersPage::parametersChanged, this, &MainWindow::syncParametersToController);
    connect(m_cameraCalibrationPage, &CameraCalibrationPage::selectImagesRequested,
            this, &MainWindow::onSelectCameraCalibrationImages);
    connect(m_cameraCalibrationPage, &CameraCalibrationPage::detectRequested,
            this, &MainWindow::onDetectCameraCalibrationImages);
    connect(m_cameraCalibrationPage, &CameraCalibrationPage::calibrateRequested,
            this, &MainWindow::onCalibrateCameraIntrinsics);
    connect(m_cameraCalibrationPage, &CameraCalibrationPage::applyRequested,
            this, &MainWindow::onApplyCameraIntrinsics);
    connect(m_cameraCalibrationPage, &CameraCalibrationPage::clearRequested,
            this, &MainWindow::onClearCameraCalibrationImages);
    connect(m_cameraCalibrationPage, &CameraCalibrationPage::goParametersRequested,
            this, [this] { navigateToPage(ParametersPageIndex); });
    connect(m_manualPosePage, &ManualPosePage::applyPointRequested,
            this, &MainWindow::onApplyManualPointInputs);
    connect(m_manualPosePage, &ManualPosePage::goParametersRequested,
            this, [this] { navigateToPage(ParametersPageIndex); });
    connect(m_manualPosePage, &ManualPosePage::goDataRequested,
            this, [this] { navigateToPage(CurrentDataPageIndex); });
    connect(m_manualPosePage, &ManualPosePage::goResultsRequested,
            this, [this] { navigateToPage(ResultPageIndex); });
    connect(m_currentDataPage, &CurrentDataPage::deleteRequested,
            m_controller, &CalibrationController::deleteSamples);
    connect(m_resultPage, &CalibrationResultPage::calculateSelectedRequested,
            this, &MainWindow::onCalculateSelected);
    connect(m_resultPage, &CalibrationResultPage::calculateAllRequested,
            this, &MainWindow::onCalculateAll);
    connect(m_resultPage, &CalibrationResultPage::computeFixedTargetRequested,
            this, &MainWindow::onComputeFixedTarget);
    connect(m_resultPage, &CalibrationResultPage::optimizeRequested,
            this, &MainWindow::onOptimizeRecommended);
    connect(m_resultPage, &CalibrationResultPage::reliabilityPipelineRequested,
            this, &MainWindow::onRunReliabilityPipeline);
    connect(m_resultPage, &CalibrationResultPage::importValidationRequested,
            this, &MainWindow::onImportValidationCsv);
    connect(m_resultPage, &CalibrationResultPage::exportRequested,
            this, &MainWindow::onExportRequested);

    connect(m_controller, &CalibrationController::samplesChanged,
            this, &MainWindow::onSamplesChanged);
    connect(m_controller, &CalibrationController::resultsChanged,
            this, &MainWindow::onResultsChanged);
    connect(m_controller, &CalibrationController::reliabilityChanged,
            this, &MainWindow::onReliabilityChanged);
    connect(m_controller, &CalibrationController::matrixChanged,
            this, &MainWindow::onMatrixChanged);
    connect(m_controller, &CalibrationController::statusChanged,
            this, &MainWindow::onStatusChanged);
    connect(m_controller, &CalibrationController::logMessage,
            this, &MainWindow::onLogMessage);
    connect(m_controller, &CalibrationController::inputSpecChanged,
            this, &MainWindow::onInputSpecChanged);
    connect(m_controller, &CalibrationController::imageProcessingFinished,
            this, &MainWindow::onImageProcessingFinished);
    connect(m_controller, &CalibrationController::cameraCalibrationChanged,
            this, &MainWindow::onCameraCalibrationChanged);
    connect(m_controller, &CalibrationController::cameraCalibrationStarted,
            this, &MainWindow::onCameraCalibrationStarted);
    connect(m_controller, &CalibrationController::cameraCalibrationFinished,
            this, &MainWindow::onCameraCalibrationFinished);
    connect(m_controller, &CalibrationController::calculationStarted,
            this, &MainWindow::onCalculationStarted);
    connect(m_controller, &CalibrationController::calculationFinished,
            this, &MainWindow::onCalculationFinished);
    connect(m_controller, &CalibrationController::reliabilityPipelineStarted,
            this, &MainWindow::onReliabilityPipelineStarted);
    connect(m_controller, &CalibrationController::reliabilityPipelineFinished,
            this, &MainWindow::onReliabilityPipelineFinished);
    connect(m_controller, &CalibrationController::reliabilityPipelineChanged,
            this, &MainWindow::onReliabilityPipelineChanged);
    connect(m_controller, &CalibrationController::error,
            this, &MainWindow::onError);
}

void MainWindow::syncParametersToController()
{
    if (!m_parametersPage || !m_controller) return;
    const PoseInputSpec spec = m_parametersPage->inputSpec();
    m_cameraCalibrationPage->setBoardSpec(m_parametersPage->boardSpec());
    m_manualPosePage->setInputSpec(spec);
    m_controller->updateInputSpec(spec, m_parametersPage->robotName(), m_parametersPage->cameraName());
    m_controller->updateImageProcessing(m_parametersPage->boardSpec(), m_parametersPage->cameraIntrinsics());
    m_controller->updateReliabilityThresholds(m_parametersPage->passRotationRmseDeg(),
                                               m_parametersPage->passTranslationRmseM());
}

void MainWindow::navigateToPage(int index)
{
    if (m_tabs && index >= 0 && index < m_tabs->count()) m_tabs->setCurrentIndex(index);
}

bool MainWindow::confirmPairing(const QString &operation)
{
    const QString text = QStringLiteral(
        "%1\n\n请确认机器人坐标和标定板图片来自同一轮采集，数量、编号和选择顺序完全一致。\n\n继续吗？")
                              .arg(operation);
    return QMessageBox::question(this, QStringLiteral("确认采集对应关系"), text,
                                 QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
           == QMessageBox::Yes;
}

void MainWindow::onNewDataset()
{
    syncParametersToController();
    m_controller->newDataset();
    navigateToPage(CapturePageIndex);
}

void MainWindow::onGenerateDemo()
{
    syncParametersToController();
    m_controller->generateDemo();
    navigateToPage(CurrentDataPageIndex);
}

void MainWindow::onImportRobotPoseCsv()
{
    syncParametersToController();
    if (!confirmPairing(QStringLiteral("即将上传机器人坐标文件。"))) return;
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("上传机器人坐标"), {},
                                                      QStringLiteral("CSV (*.csv);;文本文件 (*.txt)"));
    if (path.isEmpty()) return;
    m_controller->importRobotPoseCsv(path);
    navigateToPage(CapturePageIndex);
}

void MainWindow::onImportCalibrationImages()
{
    if (!confirmPairing(QStringLiteral("即将上传标定板图片。"))) return;
    const QStringList paths = QFileDialog::getOpenFileNames(
        this, QStringLiteral("上传标定板图片（按机器人坐标顺序选择）"), {},
        QStringLiteral("图片 (*.png *.jpg *.jpeg *.bmp)"));
    if (paths.isEmpty()) return;
    m_controller->importCalibrationImages(paths);
    navigateToPage(CapturePageIndex);
}

void MainWindow::onSelectCameraCalibrationImages()
{
    syncParametersToController();
    const QStringList paths = QFileDialog::getOpenFileNames(
        this, QStringLiteral("选择相机内参标定图片"), {},
        QStringLiteral("图片 (*.png *.jpg *.jpeg *.bmp)"));
    if (paths.isEmpty()) return;
    m_controller->importCameraCalibrationImages(paths);
    navigateToPage(CameraCalibrationPageIndex);
}

void MainWindow::onDetectCameraCalibrationImages()
{
    syncParametersToController();
    m_controller->detectCameraCalibrationImages();
}

void MainWindow::onCalibrateCameraIntrinsics()
{
    syncParametersToController();
    m_controller->calibrateCameraIntrinsics();
}

void MainWindow::onApplyCameraIntrinsics()
{
    m_controller->applyCameraIntrinsics();
}

void MainWindow::onClearCameraCalibrationImages()
{
    m_controller->clearCameraCalibrationImages();
}

void MainWindow::onApplyManualPointInputs(const QVector<PointSample> &samples,
                                           const PoseInputSpec &spec, bool calculateAll)
{
    const CalibrationDataset &dataset = m_controller->dataset();
    if (!dataset.samples.isEmpty() || !dataset.pointSamples.isEmpty()) {
        const auto answer = QMessageBox::question(
            this, QStringLiteral("替换当前训练数据"),
            QStringLiteral("当前已有训练样本。应用 FixedPoint3D 后将替换它们，是否继续？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) return;
    }
    if (!m_controller->applyManualPointInputs(samples, spec)) return;
    if (calculateAll) {
        m_controller->calculateSelected(CalibrationMethod::PointBased);
        navigateToPage(ResultPageIndex);
    } else {
        navigateToPage(ResultPageIndex);
    }
}

void MainWindow::onImportPoseImageCsv()
{
    syncParametersToController();
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入配对 CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    m_controller->importPoseImageCsv(path);
    navigateToPage(CapturePageIndex);
}

void MainWindow::onImportProcessedCsv()
{
    syncParametersToController();
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入已处理位姿 CSV"), {},
                                                      QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    m_controller->importCsv(path);
    navigateToPage(CurrentDataPageIndex);
}

void MainWindow::onImportValidationCsv()
{
    syncParametersToController();
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入独立验证 CSV"), {}, QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;
    m_controller->importValidationCsv(path);
    navigateToPage(ResultPageIndex);
}

void MainWindow::onImportJson()
{
    const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("导入 JSON"), {}, QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    m_controller->importJson(path);
    const CalibrationDataset &dataset = m_controller->dataset();
    m_parametersPage->setDatasetParameters(dataset);
    navigateToPage(CurrentDataPageIndex);
}

void MainWindow::onExportProcessedCsv()
{
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出标准 CSV"), {},
                                                      QStringLiteral("CSV (*.csv)"));
    if (!path.isEmpty()) m_controller->exportCsv(path);
}

CalibrationResult MainWindow::selectedResult() const
{
    return m_resultPage ? m_resultPage->selectedResult() : CalibrationResult{};
}

void MainWindow::onExportRequested(const QString &kind)
{
    const CalibrationResult result = selectedResult();
    if (!result.success) {
        QMessageBox::information(this, QStringLiteral("无法导出"), QStringLiteral("请先完成一次标定计算。"));
        return;
    }
    QString filter;
    if (kind == QStringLiteral("json")) filter = QStringLiteral("JSON (*.json)");
    else if (kind == QStringLiteral("yaml")) filter = QStringLiteral("YAML (*.yaml *.yml)");
    else if (kind == QStringLiteral("txt")) filter = QStringLiteral("TXT (*.txt)");
    else if (kind == QStringLiteral("cpp")) filter = QStringLiteral("C++ (*.cpp *.h)");
    else filter = QStringLiteral("Python (*.py)");
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出标定结果"), {}, filter);
    if (path.isEmpty()) return;
    const QString robot = m_parametersPage->robotName();
    Q_UNUSED(robot)
    if (kind == QStringLiteral("json")) {
        m_controller->exportJson(path);
    } else if (kind == QStringLiteral("yaml")) {
        m_controller->exportYaml(path);
    } else if (kind == QStringLiteral("txt")) {
        m_controller->exportTxt(path, result);
    } else if (kind == QStringLiteral("cpp")) {
        m_controller->exportCpp(path, result);
    } else {
        m_controller->exportPython(path, result);
    }
}

void MainWindow::onCalculateSelected()
{
    syncParametersToController();
    m_controller->calculateSelected(m_parametersPage->currentMethod());
    navigateToPage(ResultPageIndex);
}

void MainWindow::onCalculateAll()
{
    syncParametersToController();
    m_controller->calculateAll();
    navigateToPage(ResultPageIndex);
}

void MainWindow::onComputeFixedTarget(int referenceSampleId)
{
    const CalibrationResult result = selectedResult();
    if (!result.success) {
        QMessageBox::information(this, QStringLiteral("无法计算"),
                                 QStringLiteral("请先生成成功的 PosePairs 标定结果。"));
        return;
    }
    const FixedTargetPoseReport report = m_controller->computeFixedTargetPose(result, referenceSampleId);
    if (!report.success)
        QMessageBox::warning(this, QStringLiteral("fixed target pose 计算失败"), report.errors.join('\n'));
}

void MainWindow::onOptimizeRecommended()
{
    syncParametersToController();
    const CalibrationResult result = m_controller->optimizeRecommendedResult();
    if (!result.success)
        QMessageBox::warning(this, QStringLiteral("非线性优化失败"), result.message);
    navigateToPage(ResultPageIndex);
}

void MainWindow::onRunReliabilityPipeline(int bootstrapResamples, double confidenceLevel)
{
    syncParametersToController();
    m_controller->runReliabilityPipeline(bootstrapResamples, confidenceLevel);
    navigateToPage(ResultPageIndex);
}

void MainWindow::onProcessBoardImages()
{
    syncParametersToController();
    if (m_controller->processBoardImages()) navigateToPage(CurrentDataPageIndex);
}

void MainWindow::onSamplesChanged(const QVector<PoseSample> &samples)
{
    m_resultPage->setReferenceSampleIds(samples);
    QVector<PoseSample> displaySamples = samples;
    if (m_controller->dataset().inputMode == CalibrationInputMode::FixedPoint3D) {
        displaySamples.clear();
        for (const PointSample &point : m_controller->dataset().pointSamples) {
            PoseSample display;
            display.id = point.id;
            display.label = point.label;
            display.gripperRotation = point.gripperRotation;
            display.gripperTranslation = point.gripperTranslation;
            display.targetTranslation = point.cameraPoint;
            display.imageStatus = ImageSampleStatus::ManualPose;
            display.imageMessage = QStringLiteral("FixedPoint3D：相机固定点 XYZ");
            display.translationResidualM = point.residualM;
            display.outlier = point.outlier;
            displaySamples.append(display);
        }
    }
    m_currentDataPage->setSamples(displaySamples);
    int imageCount = 0;
    int targetCount = 0;
    for (const PoseSample &sample : displaySamples) {
        if (!sample.imagePath.isEmpty()) ++imageCount;
        if (sample.imageStatus == ImageSampleStatus::PoseEstimated
            || sample.imageStatus == ImageSampleStatus::ManualPose)
            ++targetCount;
    }
    m_homePage->setSummary(displaySamples.size(), imageCount, targetCount, !m_controller->dataset().results.isEmpty());
    m_capturePage->setSummary(displaySamples.size(), imageCount, targetCount);
}

void MainWindow::updateBatchSummary(const QVector<PoseSample> &samples)
{
    onSamplesChanged(samples);
}

void MainWindow::onResultsChanged(const QVector<CalibrationResult> &results)
{
    m_resultPage->setResults(results);
    m_resultPage->showPipelineReport(m_controller->dataset().reliabilityPipelineReport);
    const int sampleCount = m_controller->dataset().inputMode == CalibrationInputMode::FixedPoint3D
                                ? m_controller->dataset().pointSamples.size()
                                : m_controller->dataset().samples.size();
    m_homePage->setSummary(sampleCount,
                           std::count_if(m_controller->dataset().samples.cbegin(),
                                         m_controller->dataset().samples.cend(),
                                         [](const PoseSample &sample) { return !sample.imagePath.isEmpty(); }),
                           std::count_if(m_controller->dataset().samples.cbegin(),
                                         m_controller->dataset().samples.cend(),
                                         [](const PoseSample &sample) {
                                             return sample.imageStatus == ImageSampleStatus::PoseEstimated
                                                    || sample.imageStatus == ImageSampleStatus::ManualPose;
                                         }),
                           !results.isEmpty());
}

void MainWindow::onReliabilityChanged(const CalibrationResult &result)
{
    m_resultPage->showReliability(result);
}

void MainWindow::onMatrixChanged(const CalibrationResult &result)
{
    m_resultPage->showMatrix(result);
}

void MainWindow::onStatusChanged(const QString &text)
{
    statusBar()->showMessage(text);
    m_homePage->setStatus(text);
}

void MainWindow::onLogMessage(const QString &message)
{
    m_resultPage->appendLog(QStringLiteral("[%1] %2")
                                 .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")), message));
}

void MainWindow::onInputSpecChanged(const QString &robot, const QString &camera)
{
    m_parametersPage->setRobotCamera(robot, camera);
}

void MainWindow::onImageProcessingFinished(int processed, int succeeded)
{
    m_capturePage->setProcessingResult(processed, succeeded);
}

void MainWindow::onCameraCalibrationChanged(const CameraCalibrationReport &report)
{
    m_cameraCalibrationPage->setReport(report);
    if (report.success && report.intrinsics.valid)
        m_parametersPage->setDatasetParameters(m_controller->dataset());
}

void MainWindow::onCameraCalibrationStarted()
{
    statusBar()->showMessage(QStringLiteral("正在计算相机内参，请稍候…"));
}

void MainWindow::onCameraCalibrationFinished()
{
    statusBar()->showMessage(QStringLiteral("相机内参标定处理完成。"));
}

void MainWindow::onCalculationStarted()
{
    statusBar()->showMessage(QStringLiteral("正在计算，请稍候…"));
}

void MainWindow::onCalculationFinished()
{
    navigateToPage(ResultPageIndex);
}

void MainWindow::onReliabilityPipelineStarted()
{
    statusBar()->showMessage(QStringLiteral("正在执行可靠性流水线：质量检查、异常剔除、Huber 和 Bootstrap…"));
}

void MainWindow::onReliabilityPipelineFinished()
{
    navigateToPage(ResultPageIndex);
}

void MainWindow::onReliabilityPipelineChanged(const ReliabilityPipelineReport &report)
{
    m_resultPage->showPipelineReport(report);
}

void MainWindow::onError(const QString &title, const QString &message)
{
    QMessageBox::warning(this, title, message);
    m_homePage->setStatus(message);
}

} // namespace handeye
