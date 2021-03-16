/*
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
Shenyang Institute of Automation, Chinese Academy of Sciences.
 email: luoyang@sia.cn
*/

#include "Model.h"

#include <iostream>
//#include <boost/make_shared.hpp>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

namespace RHCL {

    Model::~Model() {
    }

    PointCloudPtr Model::getPointCloud() {
        return pcl::make_shared<PointCloud>();
    }

    PointCloudPtr Model::getPointCloud(std::vector<double> &jointRads) {
        if(jointRads.size() != _freedom) {
            std::cout << "The joints number is not equal to the freedom." << std::endl;
            return pcl::make_shared<PointCloud>();
        }

        _jntRads[0] = 0;
        for(int i = 0; i < jointRads.size(); i++) {
            _jntRads[i + 1] = jointRads[i];
        }

        PointCloudPtr pc = pcl::make_shared<PointCloud>();

        for(int i = 0; i <= _freedom; i++) {
            Eigen::Transform<double, 3, Eigen::Affine> t(Eigen::Scaling(1.0));
            for(int j = 0; j <= i; j++) {
                t *= Eigen::Translation3d(_linkGrp[j].getTranslate());

                t *= Eigen::AngleAxisd(_linkGrp[j].getRotate()[2], Eigen::Vector3d::UnitZ());
                t *= Eigen::AngleAxisd(_linkGrp[j].getRotate()[1], Eigen::Vector3d::UnitY());
                t *= Eigen::AngleAxisd(_linkGrp[j].getRotate()[0], Eigen::Vector3d::UnitX());

                t *= Eigen::AngleAxisd(_jntRads[j],_linkGrp[j].getAngleAxis());

//                t *= Eigen::Translation3d(_linkGrp[j].getTranslateLink());
//
//                t *= Eigen::AngleAxisd(_linkGrp[j].getRotateLink()[2], Eigen::Vector3d::UnitZ());
//                t *= Eigen::AngleAxisd(_linkGrp[j].getRotateLink()[1], Eigen::Vector3d::UnitY());
//                t *= Eigen::AngleAxisd(_linkGrp[j].getRotateLink()[0], Eigen::Vector3d::UnitX());

            }

            t *= Eigen::Translation3d(_linkGrp[i].getTranslateLink());

            t *= Eigen::AngleAxisd(_linkGrp[i].getRotateLink()[2], Eigen::Vector3d::UnitZ());
            t *= Eigen::AngleAxisd(_linkGrp[i].getRotateLink()[1], Eigen::Vector3d::UnitY());
            t *= Eigen::AngleAxisd(_linkGrp[i].getRotateLink()[0], Eigen::Vector3d::UnitX());

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

            if(info[j]["translateLink"].IsDefined())
                _linkGrp[j].setTranslateLink(info[j]["translateLink"][0].as<double>(),
                                         info[j]["translateLink"][1].as<double>(),
                                         info[j]["translateLink"][2].as<double>());

            if(info[j]["rotateLink"].IsDefined())
                _linkGrp[j].setRotateLink(info[j]["rotateLink"][0].as<double>(),
                                         info[j]["rotateLink"][1].as<double>(),
                                          info[j]["rotateLink"][2].as<double>());

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