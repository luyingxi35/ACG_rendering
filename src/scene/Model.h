#ifndef MODEL_H
#define MODEL_H

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <GL/glew.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>
#include <assimp/texture.h>
#include "../core/Material.h"
#include "../core/Color.h"
#include "stb_image.h"

struct ModelMesh {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> texCoords;
    std::vector<GLuint> indices;
};

class Model {
public:
    Model();
    ~Model();

    void loadModelFromFile(const std::string& path);
    std::vector<ModelMesh> meshes;
    std::vector<GLuint> textures;  // 存储纹理
    glm::mat4 transformToWorld;
    Material material;

private:
    GLuint loadTextureFromFile(const std::string& texturePath); // 处理纹理加载
    void processNode(aiNode* node, const aiScene* scene);
    void processMesh(aiMesh* mesh, const aiScene* scene, aiMaterial* material);
};

#endif // MODEL_H
