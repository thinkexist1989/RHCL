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

#include "Link.h"

#include <boost/make_shared.hpp>
#include <iostream>
#include <string>

namespace RHCL {

    Link::Link() {
        translate << 0, 0, 0;
        rotate << 0, 0, 0;
        translateLink << 0, 0, 0;
        rotateLink << 0, 0, 0;
        _pointCloud = pcl::make_shared<PointCloud>(); //new pointCloud
//        _pointCloud = boost::make_shared<PointCloud>();
//        _pointCloud = new pcl::PointCloud<pcl::PointXYZRGBA>();
    }

    Link::~Link() {

    }

    /**
     * @param fileName
     */
    Link::Link(const std::string &fileName) {
        loadAsset(fileName);
    }

    /**
     * @brief Load 3D format CAD file
     * @param fileName
     * @return return true if succeeded
     */
    bool Link::loadAsset(const std::string &fileName) {
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(fileName, aiProcess_FlipWindingOrder);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
            return false;
        }

        processNode(scene->mRootNode, scene);
        return true;
    }

    void Link::processNode(aiNode *node, const aiScene *scene) {
        for (int i = 0; i < node->mNumMeshes; ++i) {
            aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
            processMesh(mesh, scene);
        }

        for (int i = 0; i < node->mNumChildren; ++i) {
            processNode(node->mChildren[i], scene);
        }
    }

    void Link::processMesh(aiMesh *mesh, const aiScene *scene) {
        for (int i = 0; i < mesh->mNumVertices; ++i) {
            pcl::PointXYZRGBA p;
            p.x = mesh->mVertices[i].x;
            p.y = mesh->mVertices[i].y;
            p.z = mesh->mVertices[i].z;

            _pointCloud->push_back(p);
        }

    }

    Link &Link::operator=(Link &b) {
        if(this == &b)
            return *this;
        this->_pointCloud = b._pointCloud;
        this->_directory = b._directory;
        this->_order = b._order;
        return *this;
    }

    void Link::setPointCloud(const std::string &fileName) {
        _directory = fileName.substr(0, fileName.find_last_of('/'));
        std::cout << "The directory is " << _directory << std::endl;
        loadAsset(fileName); //load mesh
    }
}