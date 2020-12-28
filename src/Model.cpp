//
// Created by think on 2020/12/8.
//

#include "Model.hpp"

#include <iostream>
#include <boost/make_shared.hpp>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

namespace RHCL {

    Model::~Model() {
    }

    PointCloudPtr Model::getPointCloud() {
        return boost::make_shared<PointCloud>();
    }

    PointCloudPtr Model::getPointCloud(std::vector<double> &jointRads) {
        if(jointRads.size() != _freedom) {
            std::cout << "The joints number is not equal to the freedom." << std::endl;
            return boost::make_shared<PointCloud>();
        }

        _jntRads[0] = 0;
        for(int i = 0; i < jointRads.size(); i++) {
            _jntRads[i + 1] = jointRads[i];
        }

        PointCloudPtr pc = boost::make_shared<PointCloud>();

        for(int i = 0; i <= _freedom; i++) {
            Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::Scaling(1.0));
            for(int j = 0; j <= i; j++) {
                t *= Eigen::Translation3d(_linkGrp[j].getTranslate());
                t *= Eigen::AngleAxisd(_linkGrp[j].getRotate()[0], Eigen::Vector3d::UnitX());
                t *= Eigen::AngleAxisd(_linkGrp[j].getRotate()[1], Eigen::Vector3d::UnitY());
                t *= Eigen::AngleAxisd(_linkGrp[j].getRotate()[2], Eigen::Vector3d::UnitZ());
                t *= Eigen::AngleAxisd(_jntRads[j],_linkGrp[j].getAngleAxis());
            }

            PointCloud pc_out;
            pcl::transformPointCloud(*(_linkGrp[i].getPointCloud()), pc_out, t.matrix());
            *pc += pc_out;
        }

        return pc;
    }

    void Model::addLink(RHCL::Link link, int order) {
        if(order > _freedom) {
            std::cout << "The _order is invalid!" << std::endl;
            return;
        }
       _linkGrp[order] = link;
    }

    Model::Model(const std::string& fileName) {
        getModelFromYamlFile(fileName);
    }

    void Model::getModelFromYamlFile(const std::string &fileName) {
        //parse yaml file
        YAML::Node config = YAML::LoadFile(fileName);

        std::string dir = fileName.substr(0, fileName.find_last_of('/'));

        if(config["robot"].IsDefined()) {
            std::cout << "robot info: " << std::endl;
        }

        YAML::Node info = config["robot"];
        assert(info.Type() == YAML::NodeType::Sequence);

        std::cout << "info size is: " << info.size() << std::endl;
        std::cout << "robot freedom is " << info.size() - 1 << std::endl;

        _freedom = info.size() - 1; // The robot's freedom
        _linkGrp.resize(_freedom + 1); // the link is _freedom + 1, because of the base link4
        _jntRads.resize(_freedom + 1); // _jntRads[0] is always 0

        for (int j = 0; j < info.size(); ++j) {
            _linkGrp[j].setName(info[j]["name"].as<std::string>()); //set name
            _linkGrp[j].setOrder(info[j]["order"].as<int>()); // set order

            if(info[j]["translate"].IsDefined())
                _linkGrp[j].setTranslate(info[j]["translate"][0].as<double>(),
                                     info[j]["translate"][1].as<double>(),
                                     info[j]["translate"][2].as<double>());

            if(info[j]["rotate"].IsDefined())
                _linkGrp[j].setRotate(info[j]["rotate"][0].as<double>(),
                                  info[j]["rotate"][1].as<double>(),
                                  info[j]["rotate"][2].as<double>());

            if(info[j]["angleAxis"].IsDefined())
                _linkGrp[j].setAngleAxis(info[j]["angleAxis"][0].as<int>(),
                                         info[j]["angleAxis"][1].as<int>(),
                                         info[j]["angleAxis"][2].as<int>());

            _linkGrp[j].setPointCloud(dir+'/'+info[j]["mesh"].as<std::string>());

            std::cout << " - ";
            std::cout << "\t name: " << info[j]["name"] << std::endl;
            std::cout << "\t order: " << info[j]["order"].as<int>() << std::endl;
            if(info[j]["translate"].IsDefined())
                std::cout << "\t translate: " << info[j]["translate"][0].as<double>()
                          << ", " << info[j]["translate"][1].as<double>()
                          << ", " << info[j]["translate"][2].as<double>() << std::endl;
            if(info[j]["angleAxis"].IsDefined())
                std::cout << "\t angleAxis: " << info[j]["angleAxis"][0].as<int>()
                          << ", " << info[j]["angleAxis"][1].as<int>()
                          << ", " << info[j]["angleAxis"][2].as<int>() << std::endl;
            std::cout << "\t mesh: " << info[j]["mesh"]  << " size: " << _linkGrp[j].getPointCloudCount() << std::endl;
        }
    }

}