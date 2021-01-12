//
// Created by think on 2021/1/12.
//

#include <QApplication>
#include "RHCLVisualizer.h"
#include "mywidget.h"

int main(int argc, char** argv)
{
    QApplication a(argc, argv);

//    RHCLVisualizer rhclVisualizer;
//    rhclVisualizer.show();

    mywidget w;
    w.show();

    return  a.exec();
}

