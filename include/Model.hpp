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
        Link* _linkGrp; //Save all the links of manipulator
        double* _jntRads; // Save the joint state (radians)
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
        std::vector<Point> getPointCloud() const;

        /**
         * @brief Get the Point Cloud of the specific state
         * @param JointRads
         * @return
         */
        std::vector<Point> getPointCloud(std::vector<double> &JointRads) const;
    };

} // namespace RHCL

#endif //MODEL_H