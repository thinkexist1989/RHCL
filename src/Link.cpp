//
// Created by think on 2020/12/9.
//

#include "Link.hpp"

#include <boost/make_shared.hpp>
#include <iostream>

namespace RHCL {

    Link::Link() {

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
        const aiScene *scene = importer.ReadFile(fileName, aiProcess_Triangulate);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
            return false;
        }

        directory = fileName.substr(0, fileName.find_last_of('/'));
        std::cout << "Link directory is: " << directory << std::endl;

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
            Point p;
            p.setX(mesh->mVertices[i].x);
            p.setY(mesh->mVertices[i].y);
            p.setZ(mesh->mVertices[i].z);

            _pointCloud.push_back(p);
        }

    }
}