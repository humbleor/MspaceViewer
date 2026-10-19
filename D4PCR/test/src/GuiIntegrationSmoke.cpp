#include "OutlierRemovalRegistration.h"

#include <QApplication>
#include <QFile>
#include <QDir>
#include <QTimer>
#include <iostream>

// Runs the same asynchronous dialog path as the main application, without OSG.
// Arguments: pair <source> <target> <output>, or global <input-directory> <output>.
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    const bool global = argc == 4 && std::string(argv[1]) == "global";
    const bool pair = argc == 5 && (std::string(argv[1]) == "pair" ||
        std::string(argv[1]) == "pair-center");
    if (!global && !pair)
        return 2;

    OutlierRemovalRegistration dialog;
    auto* mode = dialog.findChild<QComboBox*>("_registrationMode");
    if (!mode || mode->count() != 2)
        return 3;
    dialog._resolution_dis->setText("0.05");
    const QString output = QString::fromLocal8Bit(argv[global ? 3 : 4]);
    dialog._outputFileOfDir->setText(output);
    if (global)
    {
        mode->setCurrentIndex(1);
        dialog.findChild<QLineEdit*>("_pointCloudDirectory")->setText(QString::fromLocal8Bit(argv[2]));
    }
    else
    {
        dialog._inputFileOfSource->setText(QString::fromLocal8Bit(argv[2]));
        dialog._inputFileOfTarget->setText(QString::fromLocal8Bit(argv[3]));
        dialog._isCenter->setChecked(std::string(argv[1]) == "pair-center");
    }

    auto* progress = new QProgressDialog(&dialog);
    QTextEdit logger;
    QTimer::singleShot(180000, &app, [&app]() { app.exit(4); });
    const bool started = dialog.executeRegistration(progress, &logger, [&]() {
        // Logs are queued to the application thread, so let them drain first.
        QTimer::singleShot(0, &app, [&]() {
            const QString logs = logger.toPlainText();
            std::cout << logs.toLocal8Bit().constData() << std::endl;
            bool passed = false;
            if (global)
                passed = QFile::exists(output + "/global_poses.csv") &&
                    QFile::exists(output + "/merged_global.pcd") &&
                    logs.contains("Global optimization completed.");
            else
                passed = !QDir(output).entryList({"*_transformationMatrix.txt"}, QDir::Files).isEmpty() &&
                    !QDir(output).entryList({"*_registered.pcd"}, QDir::Files).isEmpty() &&
                    logs.contains("4-DOF ICP completed.");
            app.exit(passed ? 0 : 5);
        });
    });
    if (!started)
        return 6;
    return app.exec();
}
