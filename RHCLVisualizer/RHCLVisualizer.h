//
// Created by think on 2021/1/12.
//

#ifndef RHCL_RHCLVISUALIZER_H
#define RHCL_RHCLVISUALIZER_H

#include <QMainWindow>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// VTK
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>

typedef  pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkRenderer> renderer2;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow2;
    PointCloudT::Ptr cloud;
    QTimer* timer;

public slots:
//    void on_pushButton_clicked();
};

#endif //RHCL_RHCLVISUALIZER_H
