#pragma once

#include <vector>
#include <GL/glew.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

class Model {
public:
    void loadModel(const std::string& path);
    void draw();  // 这里只声明函数，不定义

private:
    std::vector<GLuint> textures;
    std::vector<GLuint> VAOs;
    std::vector<GLuint> VBOs;

    GLuint loadTextureFromFile(const std::string& texturePath);
    void processNode(aiNode* node, const aiScene* scene);
    void processMesh(aiMesh* mesh, const aiScene* scene);
};
