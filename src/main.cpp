//#include "mainwindow.h"
#include "serialportwidget.h"
#include <QApplication>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SerialPortWidget w;
    w.show();

    return a.exec();
}
