//
// Created by think on 1/8/21.
//

#ifndef VTKTEST_MYWIDGET_H
#define VTKTEST_MYWIDGET_H

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class mywidget; }
QT_END_NAMESPACE

class mywidget : public QWidget {
Q_OBJECT

public:
    explicit mywidget(QWidget *parent = nullptr);

    ~mywidget() override;

private:
    Ui::mywidget *ui;
};

#endif //VTKTEST_MYWIDGET_H
