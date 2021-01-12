//
// Created by think on 2020/12/9.
//

#include "Link.hpp"

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