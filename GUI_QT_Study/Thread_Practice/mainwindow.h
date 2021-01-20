#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qdicethread.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{

    Q_OBJECT
private:
    QDiceThread threadA;

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void onthreadA_started();
    void onthreadA_finished();
    void contextMenuEvent(QContextMenuEvent *event) override;
    void onthreadA_newValue();

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
