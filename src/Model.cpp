//
// Created by think on 2020/12/8.
//

#include "Model.hpp"

#include <iostream>
#include <boost/make_shared.hpp>
#include <string>

namespace RHCL {

    Model::~Model() {
        delete[] _linkGrp;
    }

    std::vector<Point> Model::getPointCloud() const {
        return std::vector<Point>();
    }

    std::vector<Point> Model::getPointCloud(std::vector<double> &JointRads) const {
        return std::vector<Point>();
    }

    void Model::addLink(RHCL::Link link, int order) {
        if(order > _freedom) {
            std::cout << "The _order is invalid!" << std::endl;
            return;
        }
       _linkGrp[order] = link;
    }

    Model::Model(std::string &fileName) {
        getModelFromYamlFile(fileName);
    }

    void Model::getModelFromYamlFile(std::string &fileName) {
        //parse yaml file
        YAML::Node config = YAML::LoadFile("../res/config.yaml");

        if(config["robot"].IsDefined()) {
            std::cout << "robot info: " << std::endl;
        }

        YAML::Node info = config["robot"];
        assert(info.Type() == YAML::NodeType::Sequence);

        std::cout << "info size is: " << info.size() << std::endl;
        std::cout << "robot freedom is " << info.size() - 1 << std::endl;

        _freedom = info.size() - 1; // The robot's freedom
        _linkGrp = new Link[_freedom + 1]; // the link is _freedom + 1, because of the base link

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
            std::cout << "\t mesh: " << info[j]["mesh"] << std::endl;
        }
    }

}