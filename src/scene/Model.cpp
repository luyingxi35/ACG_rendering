#include "Model.h"
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>  // stb_image 用于加载图片文件

Model::Model() {}

Model::~Model() {
    // 清理加载的纹理
    for (GLuint texture : textures) {
        glDeleteTextures(1, &texture);
    }
}

void Model::loadModelFromFile(const std::string& path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipWindingOrder);
    if (!scene || !scene->mRootNode) {
        std::cerr << "Error loading model: " << importer.GetErrorString() << std::endl;
        return;
    }
    processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode* node, const aiScene* scene) {
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        processMesh(mesh, scene);
    }

    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene);
    }
}

void Model::processMesh(aiMesh* mesh, const aiScene* scene) {
    ModelMesh modelMesh;

    // 获取顶点、法线、纹理坐标和索引
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        aiVector3D pos = mesh->mVertices[i];
        aiVector3D normal = mesh->mNormals[i];
        aiVector3D texCoord = mesh->mTextureCoords[0] ? mesh->mTextureCoords[0][i] : aiVector3D(0.0f, 0.0f, 0.0f);

        modelMesh.vertices.push_back(glm::vec3(pos.x, pos.y, pos.z));
        modelMesh.normals.push_back(glm::vec3(normal.x, normal.y, normal.z));
        modelMesh.texCoords.push_back(glm::vec2(texCoord.x, texCoord.y));
    }

    // 获取面（索引）
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++) {
            modelMesh.indices.push_back(face.mIndices[j]);
        }
    }

    meshes.push_back(modelMesh);

    std::cout << "Processed mesh with " << mesh->mNumVertices << " vertices and "
        << mesh->mNumFaces * 3 << " indices." << std::endl;

    // 加载材质中的纹理（如果有）
    if (mesh->mMaterialIndex >= 0) {
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
        aiString texturePath;
        if (material->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath) == AI_SUCCESS) {
            GLuint texture = loadTextureFromFile(texturePath.C_Str());
            textures.push_back(texture);  // 添加到纹理列表
        }
    }
}

GLuint Model::loadTextureFromFile(const std::string& texturePath) {
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // 使用 stb_image 库加载图片文件
    int width, height, nrChannels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
    if (data) {
        GLenum format = (nrChannels == 3) ? GL_RGB : GL_RGBA; // 判断图片通道数
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else {
        std::cerr << "Texture failed to load at path: " << texturePath << std::endl;
        return -1;
    }
    stbi_image_free(data);

    return textureID;
}

void Model::draw() const {
    std::cout << "Drawing model with " << meshes.size() << " meshes" << std::endl;
    for (size_t i = 0; i < meshes.size(); ++i) {
        const ModelMesh& mesh = meshes[i];
        std::cout << "Mesh " << i + 1 << " with " << mesh.vertices.size() << " vertices and "
            << mesh.indices.size() << " indices." << std::endl;
    }

    for (GLuint texture : textures) {
        std::cout << "Texture ID: " << texture << std::endl;
    }
}
