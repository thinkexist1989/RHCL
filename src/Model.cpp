//
// Created by think on 2020/12/8.
//

#include "Model.hpp"

namespace RHCL {
    Model::Model(/* args */) {
    }

    Model::~Model() {
    }

    std::vector<Point> Model::getPointCloud() const {
        return std::vector<Point>();
    }

    std::vector<Point> Model::getPointCloud(std::vector<double> &JointRads) const {
        return std::vector<Point>();
    }

}