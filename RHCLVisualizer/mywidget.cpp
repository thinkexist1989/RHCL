//
// Created by think on 1/8/21.
//

// You may need to build the project (run Qt uic code generator) to get "ui_mywidget.h" resolved

#include "mywidget.h"
#include "ui_mywidget.h"

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

mywidget::mywidget(QWidget *parent) :
        QWidget(parent), ui(new Ui::mywidget) {
    ui->setupUi(this);

    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();

    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(sphereMapper);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(sphereActor);

    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

}

mywidget::~mywidget() {
    delete ui;
}
