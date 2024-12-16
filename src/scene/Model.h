#ifndef MODEL_H
#define MODEL_H

#include <GL/glew.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>
#include <assimp/texture.h>
#include "../core/Material.h"
#include "../core/Color.h"
#include "../core/Texture.h"
#include "../utils/definition.h"
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
    std::vector<ModelMesh> meshes;
    std::vector<Triangle> triangles;
    glm::mat4 transformToWorld;
	Material material; //纹理在material中
private:
    //GLuint loadTextureFromFile(const std::string& texturePath); // 处理纹理加载
    void processNode(aiNode* node, const aiScene* scene);
    void processMesh(aiMesh* mesh, const aiScene* scene);
    //MipmapTexture loadTextureAndGenerateMipmap(const std::string& texturePath);
};

#endif // MODEL_H
