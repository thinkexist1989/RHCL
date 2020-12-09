#ifndef LINK_H
#define LINK_H

#include "Point.hpp"
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace RHCL {
    class Link {
    private:
        /* data */
        std::vector<Point> _pointCloud; //store the point cloud of this link

        std::string directory; // the directory of CAD file
    public:
        Link(/* args */);

        Link(const std::string &fileName);

        ~Link();

        bool loadAsset(const std::string &fileName); //load 3D format CAD file

        inline const std::vector<Point>& getPointCloud() const {return  _pointCloud;} //get the point cloud of link

    private:
        void processNode(aiNode *node, const aiScene *scene);

        void processMesh(aiMesh *mesh, const aiScene *scene);

    };

} // namespace RHCL

#endif // LINK_H