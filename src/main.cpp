#include "mainwindow.h"

#include "core/calibration_service.h"
#include "core/synthetic_dataset.h"
#include "io/dataset_io.h"

#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QTemporaryDir>
#include <QLocale>
#include <QTranslator>

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
        for (const handeye::CalibrationResult &result : results) {
            successCount += result.success ? 1 : 0;
            smokeStream << handeye::methodName(result.method) << "|" << result.success << "|"
                        << result.message << Qt::endl;
            qInfo().noquote() << handeye::methodName(result.method) << result.message;
        }
        smokeStream << "success=" << successCount << "/" << handeye::allMethods().size() << Qt::endl;
        QTemporaryDir tempDir;
        handeye::CalibrationDataset csvDataset;
        handeye::CalibrationDataset jsonDataset;
        const auto csvWrite = handeye::writeCsv(tempDir.filePath(QStringLiteral("samples.csv")), dataset);
        const auto csvRead = handeye::readCsv(tempDir.filePath(QStringLiteral("samples.csv")), &csvDataset);
        const auto jsonWrite = handeye::writeJson(tempDir.filePath(QStringLiteral("samples.json")), dataset);
        const auto jsonRead = handeye::readJson(tempDir.filePath(QStringLiteral("samples.json")), &jsonDataset);
        const bool ioOk = csvWrite.success && csvRead.success && jsonWrite.success && jsonRead.success
                          && csvDataset.samples.size() == dataset.samples.size()
                          && jsonDataset.samples.size() == dataset.samples.size();
        smokeStream << "io_roundtrip=" << ioOk << Qt::endl;
        smokeStream << "csv_write=" << csvWrite.success << ",csv_read=" << csvRead.success
                    << ",json_write=" << jsonWrite.success << ",json_read=" << jsonRead.success
                    << ",csv_count=" << csvDataset.samples.size() << ",json_count=" << jsonDataset.samples.size()
                    << Qt::endl;
        qInfo() << "success" << successCount << "/" << handeye::allMethods().size();
        return successCount == handeye::allMethods().size() && ioOk ? 0 : 1;
    }
    handeye::MainWindow window;
    window.show();
    return app.exec();
}
