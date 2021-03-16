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

#ifndef LINK_H
#define LINK_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Dense>

#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZRGBA

namespace RHCL {
    typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

    class Link {
    private:
        /* data */
        PointCloudPtr _pointCloud; //store the point cloud of this link

        std::string _directory; // the _directory of CAD file

        int _order = 0; //the _order of the link

        std::string _name; // the name of the link

        Eigen::Vector3d translate; //translate relative to previous joint
        Eigen::Vector3d rotate; // rotate relative to previous joint
        Eigen::Vector3d angleAxis; // angle-axis format

        Eigen::Vector3d translateLink; //translate the link
        Eigen::Vector3d rotateLink;
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
        inline Eigen::Vector3d getAngleAxis() {return angleAxis;}

        inline void setTranslateLink(double x, double y, double z) {translateLink << x, y, z; }
        inline Eigen::Vector3d getTranslateLink() {return translateLink;}

        inline  void setRotateLink(double roll, double pitch, double yaw) {rotateLink << roll, pitch, yaw; }
        inline Eigen::Vector3d getRotateLink() {return rotateLink;}

        /**
         * @details Generate the point cloud from mesh
         * @param fileName
         */
        void setPointCloud(const std::string& fileName);
//        inline std::vector<Point> getPointCloud() const {return _pointCloud;} // get the point cloud
        inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud() const {return  _pointCloud;} //get the point cloud of link
        inline const int getPointCloudCount() const { return _pointCloud->size();} //Get the size of the point cloud


        bool loadAsset(const std::string &fileName); //load 3D format CAD file

        Link& operator=(Link &b); //overload assignment
    private:
        void processNode(aiNode *node, const aiScene *scene);

        void processMesh(aiMesh *mesh, const aiScene *scene);

    };

} // namespace RHCL

#endif // LINK_H