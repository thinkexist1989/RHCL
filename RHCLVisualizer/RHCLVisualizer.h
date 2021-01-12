//
// Created by think on 2021/1/12.
//

#ifndef RHCL_RHCLVISUALIZER_H
#define RHCL_RHCLVISUALIZER_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class RHCLVisualizer; }
QT_END_NAMESPACE

class RHCLVisualizer : public QMainWindow {
Q_OBJECT

public:
    explicit RHCLVisualizer(QWidget *parent = nullptr);

    ~RHCLVisualizer() override;

private:
    Ui::RHCLVisualizer *ui;
};

#endif //RHCL_RHCLVISUALIZER_H
