//
// Created by think on 2021/1/13.
//

// You may need to build the project (run Qt uic code generator) to get "ui_MyVTKOpenGLWidget.h" resolved

#include "myvtkopenglwidget.h"
#include "ui_MyVTKOpenGLWidget.h"

#include <vtkPointPicker.h>
#include <vtkGenericOpenGLRenderWindow.h>

MyVTKOpenGLWidget::MyVTKOpenGLWidget(QWidget *parent) :
        QVTKOpenGLWidget(parent), ui(new Ui::MyVTKOpenGLWidget) {
    ui->setupUi(this);

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    _renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    _renderWindow->AddRenderer(renderer);
    _viewer.reset(new pcl::visualization::PCLVisualizer(renderer, _renderWindow, "viewer", false));
    this->SetRenderWindow(_viewer->getRenderWindow());
    this->update();
}

MyVTKOpenGLWidget::~MyVTKOpenGLWidget() {
    delete ui;
}

void MyVTKOpenGLWidget::populateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    if (!_viewer->updatePointCloud(cloud, "cloud")) {
        _viewer->addPointCloud(cloud, "cloud");
    }
    _renderWindow->Render();
}
