#ifndef MODEL_H
#define MODEL_H

#include "Link.hpp"
#include <vector>
#include <boost/shared_ptr.hpp>

namespace RHCL {
    class Model {
    public:
        typedef Link Link; //typedef in Model, so can be used as RHCL::Model::Link

    private:
        /* data */
        std::vector<Link> _linkPtrGrp; //Save all the links of manipulator

    public:
        Model(/* args */);
        
        ~Model();

        /**
         * @brief Get the number of links
         * 
         * @return int 
         */
        inline int getLinkNum() const { return _linkPtrGrp.size(); }

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