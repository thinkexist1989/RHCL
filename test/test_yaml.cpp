//
// Created by think on 2020/12/17.
//

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include "boost/date_time.hpp"

using namespace boost;

int main(int argc, char** argv)
{
    YAML::Node config = YAML::LoadFile("config.yaml");

    if(config["robotinfo"].IsDefined()) {
        std::cout << "robot info: " << std::endl;
    }

    YAML::Node info = config["robotinfo"];
    assert(info.Type() == YAML::NodeType::Sequence);

    std::cout << "info size is: " << info.size() << std::endl;

    for (int j = 0; j < info.size(); ++j) {
        std::cout << " - ";
        std::cout << "\t name: " << info[0]["name"] << std::endl;
        std::cout << "\t parent: " << info[0]["parent"] << std::endl;
        std::cout << "\t mesh: " << info[0]["mesh"] << std::endl;
    }

    return 0;
}