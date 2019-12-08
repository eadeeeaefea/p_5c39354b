#include "include/mainwindow.h"
#include <QApplication>
#include <iostream>

int main(int argc, char *argv[])
{
    const char* serial_name = "/dev/ttyUSB1";
    if(argc == 2)
    {
        serial_name = argv[1];
    }

    std::cout << "serial name: " << serial_name << std::endl;

    QApplication a(argc, argv);
    MainWindow w(serial_name);
    w.show();
    w.run();

    return a.exec();
}
