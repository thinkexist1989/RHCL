#ifndef MODEL_H
#define MODEL_H

#include "Link.hpp"
#include <vector>
#include "yaml-cpp/yaml.h"

namespace RHCL {
    class Model {
    public:
//        typedef Link Link; //typedef in Model, so can be used as RHCL::Model::Link

    private:
        /* data */
        std::vector<Link> _linkGrp; //Save all the links of manipulator
        std::vector<double> _jntRads; // Save the joint state (radians)
        int _freedom = 0;

    public:
        /**
         *
         * @param fileName The yaml file
         */
        Model(const std::string& fileName);
        
        ~Model();

        void getModelFromYamlFile(const std::string& fileName); // Load model from file

        void addLink(RHCL::Link link, int order);

        /**
         * @brief Get the number of links
         * 
         * @return int 
         */
        inline int getFreedom() const { return _freedom; }

        /**
         * @brief Get the Point Cloud of current state
         * 
         * @return std::vector<Point> 
         */
        PointCloudPtr getPointCloud();

        /**
         * @brief Get the Point Cloud of the specific state
         * @param jointRads
         * @return
         */
        PointCloudPtr getPointCloud(std::vector<double> &jointRads);
    };

} // namespace RHCL

#endif //MODEL_H