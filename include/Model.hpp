#ifndef MODEL_H
#define MODEL_H

#include "Link.hpp"
#include <vector>
#include <boost/shared_ptr.hpp>

namespace RHCL
{
    class Model
    {
    public:
        typedef Link Link; //typedef in Model, so can be used as RHCL::Model::Link

    private:
        /* data */
        std::vector<boost::shared_ptr<Link>> _linkPtrs; //Save all the links of manipulaotr

    public:
        Model(/* args */);
        ~Model();

        /**
         * @brief Get the number of links
         * 
         * @return int 
         */
        inline int getLinkNum() const { return _linkPtrs.size(); }

        /**
         * @brief Get the Point Cloud of current state
         * 
         * @return std::vector<Point> 
         */
        std::vector<Point> getPointCloud() const;

        std::vector<Point> getPointCloud(std::vector<double> &JointRads) const;
    };

    Model::Model(/* args */)
    {
    }

    Model::~Model()
    {
    }

} // namespace RHCL

#endif //MODEL_H