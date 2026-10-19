#pragma once
#include "AsyncRegistration.h"
#include <QCoreApplication>
#include <QProcess>
#include <QProcessEnvironment>
#include <QFileInfo>
#include <QMessageBox>
#include <memory>

namespace registrationui {
// A child process makes blocking point-cloud calls safely stoppable using X.
inline bool startProcess(QProgressDialog* progress,QTextEdit* logger,
                         const QString& executable,const QStringList& arguments,
                         const QByteArray& progressPrefix,const QString& output,
                         std::function<void()> onFinished) {
    if(!QFileInfo::exists(executable)) {
        QMessageBox::warning(progress,QObject::tr("配准失败"),QObject::tr("缺少全局配准程序，请重新编译：")+executable);return false;
    }
    struct State {bool canceled=false,finished=false;QByteArray pending;};
    auto state=std::make_shared<State>();QPointer<QProgressDialog> guard(progress);QPointer<QTextEdit> logGuard(logger);
    auto* process=new QProcess(progress);process->setProcessChannelMode(QProcess::MergedChannels);
    process->setWorkingDirectory(QCoreApplication::applicationDirPath());
    auto environment=QProcessEnvironment::systemEnvironment();
    environment.insert("PATH",QCoreApplication::applicationDirPath()+";"+environment.value("PATH"));process->setProcessEnvironment(environment);
    progress->setRange(0,100);progress->setValue(0);progress->setCancelButton(nullptr);
    progress->setWindowFlags(progress->windowFlags()|Qt::WindowCloseButtonHint);
    progress->setAutoClose(false);progress->setAutoReset(false);progress->setWindowModality(Qt::NonModal);
    progress->setLabelText(QObject::tr("正在启动 Forest 全局配准……"));
    auto consume=[state,guard,logGuard,progressPrefix](const QByteArray& data,bool flush){
        state->pending+=data;
        while(!state->pending.isEmpty()) {
            auto newline=state->pending.indexOf('\n');if(newline<0&&!flush)break;
            auto line=(newline<0?state->pending:state->pending.left(newline)).trimmed();
            state->pending.remove(0,newline<0?state->pending.size():newline+1);
            if(line.startsWith(progressPrefix)) {
                auto separator=line.indexOf('\t',progressPrefix.size());bool ok=false;
                int value=line.mid(progressPrefix.size(),separator-progressPrefix.size()).toInt(&ok);
                if(separator<0||!ok)continue;auto message=QString::fromUtf8(line.mid(separator+1));
                if(guard&&!state->canceled){guard->setValue(value);guard->setLabelText(message);}
                log(logGuard,message+"\n",true);
            }else if(!line.isEmpty())log(logGuard,QString::fromUtf8(line)+"\n",true);
        }
    };
    auto finish=[state,guard,logGuard,process,consume,output,onFinished=std::move(onFinished)](int code,QProcess::ExitStatus status){
        if(state->finished)return;state->finished=true;consume(process->readAllStandardOutput(),true);
        if(state->canceled)log(logGuard,QObject::tr("配准已终止，尚未完成的输出文件不可作为成功结果使用。\n"));
        else if(status!=QProcess::NormalExit)log(logGuard,QObject::tr("配准进程异常退出：")+process->errorString()+"\n");
        else if(code!=0)log(logGuard,QObject::tr("全局配准失败，退出代码：%1\n").arg(code));
        else log(logGuard,QObject::tr("全局配准结果已保存至：")+output+"\n");
        if(guard){guard->close();guard->deleteLater();}if(onFinished)onFinished();
    };
    QObject::connect(process,&QProcess::readyReadStandardOutput,progress,[process,consume](){consume(process->readAllStandardOutput(),false);});
    QObject::connect(process,&QProcess::finished,progress,finish);
    QObject::connect(process,&QProcess::errorOccurred,progress,[finish](QProcess::ProcessError error){if(error==QProcess::FailedToStart)finish(-1,QProcess::CrashExit);});
    QObject::connect(process,&QProcess::started,progress,[state,process](){if(state->canceled)process->kill();});
    QObject::connect(progress,&QProgressDialog::canceled,progress,[state,guard,process](){
        if(state->finished||state->canceled)return;state->canceled=true;
        if(guard){guard->setLabelText(QObject::tr("正在终止配准……"));guard->show();}process->kill();
    });
    process->start(executable,arguments);progress->show();return true;
}
} // namespace registrationui
