#include "volturnus_gui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    volturnus_gui w;
    w.show();

    return a.exec();
}
