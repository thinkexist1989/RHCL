//
// Created by think on 2021/1/13.
//

#ifndef RHCL_PROJECT_MYVTKOPENGLWIDGET_H
#define RHCL_PROJECT_MYVTKOPENGLWIDGET_H

#include <QWidget>
#include <QVTKOpenGLWidget.h>
#include <pcl/visualization//pcl_visualizer.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MyVTKOpenGLWidget; }
QT_END_NAMESPACE

class MyVTKOpenGLWidget : public QVTKOpenGLWidget {
Q_OBJECT

public:
    explicit MyVTKOpenGLWidget(QWidget *parent = nullptr);

    ~MyVTKOpenGLWidget() override;

    void populateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

private:
    Ui::MyVTKOpenGLWidget *ui;

    pcl::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> _renderWindow;
};

#endif //RHCL_PROJECT_MYVTKOPENGLWIDGET_H
