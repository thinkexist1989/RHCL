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

RHCLVisualizer::RHCLVisualizer(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::RHCLVisualizer) {
    ui->setupUi(this);

//    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
//
//    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
//
//    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
//    sphereActor->SetMapper(sphereMapper);
//
//    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
//    renderer->AddActor(sphereActor);
//
//    ui->vtkModelWidget->GetRenderWindow()->AddRenderer(renderer);
//    ui->vtkKinectWidget->GetRenderWindow()->AddRenderer(renderer);
}

RHCLVisualizer::~RHCLVisualizer() {
    delete ui;
}
