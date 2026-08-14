#pragma once

#include <QObject>
#include <QFuture>
#include <QFutureWatcher>
#include <QEventLoop>

// Runs a QFuture to completion by spinning a local event loop instead of the
// former `while (!future.isFinished()) QApplication::processEvents();` busy-wait.
// This keeps the GUI (and any shown progress dialog) responsive without burning
// a CPU core, and is behavior-identical: the caller still blocks until the
// background task finishes.
template <typename T>
inline void runFutureBlocking(const QFuture<T>& future)
{
    QEventLoop loop;
    QFutureWatcher<T> watcher;
    QObject::connect(&watcher, &QFutureWatcher<T>::finished, &loop, &QEventLoop::quit);
    watcher.setFuture(future);
    // QFutureWatcher delivers finished() through a queued connection even when the
    // future is already done, so this never deadlocks.
    if (!future.isFinished())
        loop.exec();
}
