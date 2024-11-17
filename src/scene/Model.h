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
    Material material_;
};

class Model {
public:
    Model();
    ~Model();

    void loadModelFromFile(const std::string& path);
    void draw() const;  // for debugging
    std::vector<ModelMesh> meshes;
    std::vector<GLuint> textures;  // �洢����

private:
    GLuint loadTextureFromFile(const std::string& texturePath); // ������������
    void processNode(aiNode* node, const aiScene* scene);
    void processMesh(aiMesh* mesh, const aiScene* scene, aiMaterial* material);
    Material loadMaterial(aiMaterial* material);
};

#endif // MODEL_H