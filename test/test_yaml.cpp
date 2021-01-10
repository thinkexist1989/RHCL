//
// Created by think on 2020/12/17.
//

#include <yaml-cpp/yaml.h>
#include <iostream>
#include "Model.hpp"
#include "Link.hpp"
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    YAML::Node config = YAML::LoadFile("../res/iiwa/config.yaml");

    if(config["robot"].IsDefined()) {
        std::cout << "robot info: " << std::endl;
    }

    YAML::Node info = config["robot"];
    assert(info.Type() == YAML::NodeType::Sequence);

    std::cout << "info size is: " << info.size() << std::endl;
    std::cout << "robot freedom is " << info.size() - 1 << std::endl;

    RHCL::Model model("../res/iiwa/config.yaml");

    std::vector<double> jntRads(model.getFreedom(), 0);

    RHCL::PointCloudPtr pc = model.getPointCloud(jntRads);

    std::cout << "Point cloud size is: " << pc->size() << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer1"));

    //source点云窗口
    viewer1->setBackgroundColor(255, 255, 255);//设置背景色为白色
    viewer1->addText("source_point_cloud_image", 10, 10,1.0,0.0,0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> target_color(pc, 255, 0, 0);
    viewer1->addPointCloud(pc, target_color, "source_point_cloud");

    while (!viewer1->wasStopped())
    {
        viewer1->spinOnce(100);   //100??
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
//        std::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

//    for (int j = 0; j < info.size(); ++j) {
//        std::cout << " - ";
//        std::cout << "\t name: " << info[j]["name"] << std::endl;
//        std::cout << "\t order: " << info[j]["order"].as<int>() << std::endl;
//        if(info[j]["translate"].IsDefined())
//            std::cout << "\t translate: " << info[j]["translate"][0].as<double>()
//                    << ", " << info[j]["translate"][1].as<double>()
//                    << ", " << info[j]["translate"][2].as<double>() << std::endl;
//        if(info[j]["angleAxis"].IsDefined())
//            std::cout << "\t angleAxis: " << info[j]["angleAxis"][0].as<int>()
//                      << ", " << info[j]["angleAxis"][1].as<int>()
//                      << ", " << info[j]["angleAxis"][2].as<int>() << std::endl;
//        std::cout << "\t mesh: " << info[j]["mesh"] << std::endl;
//    }

    return 0;
}