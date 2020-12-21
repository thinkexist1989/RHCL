//
// Created by think on 2020/12/17.
//

#include <yaml-cpp/yaml.h>
#include <iostream>
#include "Model.hpp"
#include "Link.hpp"

int main(int argc, char** argv)
{
    YAML::Node config = YAML::LoadFile("../res/config.yaml");

    if(config["robotinfo"].IsDefined()) {
        std::cout << "robot info: " << std::endl;
    }

    YAML::Node info = config["robot"];
    assert(info.Type() == YAML::NodeType::Sequence);

    std::cout << "info size is: " << info.size() << std::endl;
    std::cout << "robot freedom is " << info.size() - 1 << std::endl;

    for (int j = 0; j < info.size(); ++j) {
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

    return 0;
}