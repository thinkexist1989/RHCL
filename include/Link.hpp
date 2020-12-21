#ifndef LINK_H
#define LINK_H

#include "Point.hpp"
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Dense>

namespace RHCL {
    class Link {
    private:
        /* data */
        std::vector<Point> _pointCloud; //store the point cloud of this link

        std::string _directory; // the _directory of CAD file

        int _order = 0; //the _order of the link

        std::string _name; // the name of the link

        Eigen::Vector3d translate; //translate relative to previous link
        Eigen::Vector3d rotate; // rotate relative to previous link
        Eigen::Vector3i angleAxis; // angle-axis format
    public:
        Link(/* args */);

        Link(const std::string &fileName);

        ~Link();

        inline void setName(const std::string& name) {_name = name;}
        inline std::string getName() {return _name;}

        inline void setOrder(int order) {_order = order;}
        inline int getOrder() {return _order;}

        inline void setTranslate(double x, double y, double z) {translate << x, y, z; }
        inline Eigen::Vector3d getTranslate() {return translate;}

        inline  void setRotate(double roll, double pitch, double yaw) {rotate << roll, pitch, yaw; }
        inline Eigen::Vector3d getRotate() {return rotate;}

        inline void setAngleAxis(int x, int y, int z) {angleAxis << x, y, z;}
        inline Eigen::Vector3i getAngleAxis() {return angleAxis;}

        /**
         * @details Generate the point cloud from mesh
         * @param fileName
         */
        void setPointCloud(const std::string& fileName);
//        inline std::vector<Point> getPointCloud() const {return _pointCloud;} // get the point cloud
        inline const std::vector<Point>& getPointCloud() const {return  _pointCloud;} //get the point cloud of link
        inline const int getPointCloudCount() const { return _pointCloud.size();} //Get the size of the point cloud


        bool loadAsset(const std::string &fileName); //load 3D format CAD file

        Link& operator=(Link &b); //overload assignment
    private:
        void processNode(aiNode *node, const aiScene *scene);

        void processMesh(aiMesh *mesh, const aiScene *scene);

    };

} // namespace RHCL

#endif // LINK_H