//
// Created by think on 2021/1/12.
//

// You may need to build the project (run Qt uic code generator) to get "ui_RHCLVisualizer.h" resolved

#include "RHCLVisualizer.h"
#include "ui_RHCLVisualizer.h"

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include <iostream>

RHCLVisualizer::RHCLVisualizer(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::RHCLVisualizer) {
    ui->setupUi(this);

    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();

    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(sphereMapper);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(sphereActor);

    ui->vtkModelWidget->GetRenderWindow()->AddRenderer(renderer);
//    ui->vtkKinectWidget->GetRenderWindow()->AddRenderer(renderer);

    cloud.reset(new PointCloudT);
    cloud->points.resize(200);

    // Fill the cloud with some points
    for (auto& point: *cloud)
    {
        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
        point.z = 1024 * rand () / (RAND_MAX + 1.0f);

        point.r = 128;
        point.g = 128;
        point.b = 128;
    }

    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    renderer2->GradientBackgroundOn();
    renderer2->SetBackground(0.27,0.27,0.27);
    renderer2->SetBackground2(0.44,0.44,0.44);
    renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
    ui->vtkKinectWidget->SetRenderWindow(viewer->getRenderWindow());

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> target_color(cloud, 255, 0, 0);
//    viewer->setupInteractor(ui->vtkKinectWidget->GetInteractor(), ui->vtkKinectWidget->GetRenderWindow());
//    ui->vtkKinectWidget->update();

    viewer->addPointCloud(cloud, "cloud");
//    viewer->resetCamera();
    renderWindow2->Render();

    ui->vtkKinectWidget->update();
}

RHCLVisualizer::~RHCLVisualizer() {
    delete ui;
}

void RHCLVisualizer::on_pushButton_clicked() {
    std::cout << cloud->size() << std::endl;

    // Set the new color
    for (auto& point: *cloud)
    {
        point.r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
        point.g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
        point.b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    }
    viewer->spinOnce(100);
    viewer->updatePointCloud (cloud, "cloud");
    renderWindow2->Render();
//    ui->vtkKinectWidget->update ();


}
