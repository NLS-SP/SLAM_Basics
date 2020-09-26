#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDialog>
#include <QFileDialog>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->menubar->setNativeMenuBar(false);

    // 点击新建按钮，弹出对话框
    connect(ui->actionCreate, &QAction::triggered, [=](){
        // 弹出对话框
        // 模态对话框->不可以对其他窗口进行操作（阻塞功能）
        // 非模态对话框->可以对其他窗口进行操作
        // QDialog dlg(this);
        // dlg.exec();

        // 创建非模态对话框
        // QDialog *dlg2 = new QDialog(this);
        // dlg2->show();
        // dlg2->setAttribute(Qt::WA_DeleteOnClose);

        // 文件对话框 参数1 父亲 参数2 标题 参数3 打开文件地址 参数4 过滤文件格式
        QString str = QFileDialog::getOpenFileName(this, "Open File", "~/", "(*.txt)");
        qDebug() << str;
    }

    );


}

MainWindow::~MainWindow()
{
    delete ui;
}

