#include "parameters_vector_manager.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ParametersVectorManager w;
    w.show();

    return a.exec();
}
