#pragma once

#include <QCoreApplication>
#include <QFutureWatcher>
#include <QPointer>
#include <QProgressDialog>
#include <QTextEdit>
#include <QtConcurrent/QtConcurrent>

#include <functional>
#include <utility>

namespace registrationui {

using ProgressReporter = std::function<void(int, const QString&)>;

inline void log(QPointer<QTextEdit> logger, const QString& message, bool scroll = false)
{
    QMetaObject::invokeMethod(QCoreApplication::instance(), [logger, message, scroll]() {
        if (!logger)
            return;
        logger->insertPlainText(message);
        if (scroll)
            logger->ensureCursorVisible();
    }, Qt::QueuedConnection);
}

template <typename Work>
void start(QProgressDialog* progress, QTextEdit* logger, const QString& initialStage,
           std::function<void()> onFinished, Work work)
{
    progress->setRange(0, 100);
    progress->setValue(0);
    progress->setLabelText(initialStage);
    progress->setCancelButton(nullptr);
    progress->setWindowFlags(progress->windowFlags() & ~Qt::WindowCloseButtonHint);
    progress->setAutoClose(false);
    progress->setWindowModality(Qt::NonModal);
    progress->show();

    QPointer<QProgressDialog> progressGuard(progress);
    QPointer<QTextEdit> loggerGuard(logger);
    ProgressReporter report = [progressGuard](int value, const QString& stage) {
        QMetaObject::invokeMethod(QCoreApplication::instance(), [progressGuard, value, stage]() {
            if (progressGuard) {
                progressGuard->setLabelText(stage);
                progressGuard->setValue(value);
            }
        }, Qt::QueuedConnection);
    };

    auto* watcher = new QFutureWatcher<void>(progress);
    QObject::connect(watcher, &QFutureWatcher<void>::finished, progress,
                     [progressGuard, onFinished = std::move(onFinished)]() {
        if (progressGuard) {
            progressGuard->setValue(100);
            progressGuard->close();
            progressGuard->deleteLater();
        }
        onFinished();
    });
    watcher->setFuture(QtConcurrent::run([work = std::move(work), loggerGuard, report]() mutable {
        work(loggerGuard, report);
    }));
}

} // namespace registrationui
