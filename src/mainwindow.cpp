#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "serialportwidget.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    SerialPortWidget *serialPortWidget = new SerialPortWidget(this);
    ui->tabWidget->addTab(serialPortWidget, "SerialPort");
}

MainWindow::~MainWindow()
{
    delete ui;
}
