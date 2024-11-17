#include "Model.h"
#include <iostream>

Model::Model() {}
Model::~Model() {}

void Model::loadModelFromFile(const std::string& path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cerr << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        return;
    }

    // ���������ڵ�
    processNode(scene->mRootNode, scene);
    std::cout << "Loading a model finished." << std::endl;
}

void Model::processNode(aiNode* node, const aiScene* scene) {
    // ������ǰ�ڵ����������
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        aiMaterial* material = scene->mMaterials[scene->mMeshes[node->mMeshes[i]]->mMaterialIndex];
        processMesh(mesh, scene, material);
    }

    // �ݹ鴦���ӽڵ�
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene);
    }
}

void Model::processMesh(aiMesh* mesh, const aiScene* scene, aiMaterial* material) {
    ModelMesh modelMesh;

    // ��ȡ��������
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        glm::vec3 vertex(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        glm::vec3 normal(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
        glm::vec2 texCoord(0.0f, 0.0f);

        if (mesh->mTextureCoords[0]) {
            texCoord = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
        }

        modelMesh.vertices.push_back(vertex);
        modelMesh.normals.push_back(normal);
        modelMesh.texCoords.push_back(texCoord);
    }

    // ��ȡ��������
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++) {
            modelMesh.indices.push_back(face.mIndices[j]);
        }
    }

    // ��������
    modelMesh.material_ = loadMaterial(material);

    meshes.push_back(modelMesh);
}

Material Model::loadMaterial(aiMaterial* aiMat) {
    Material material;

    aiColor3D color(0.f, 0.f, 0.f);

    // ��ȡ��������ɫ
    if (aiMat->Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS) {
        material.diffuseColor = Color(color.r, color.g, color.b);
    }
    else {
        std::cout << "No diffuse color found for mesh " << std::endl;
    }

    // ��ȡ���淴����ɫ
    if (aiMat->Get(AI_MATKEY_COLOR_SPECULAR, color) == AI_SUCCESS) {
        material.specularColor = Color(color.r, color.g, color.b);
    }
    else {
        std::cout << "No specular color found for mesh " << std::endl;
    }

    // ��ȡ�����
    float shininess;
    if (aiMat->Get(AI_MATKEY_SHININESS, shininess) == AI_SUCCESS) {
        material.shineness = shininess;
    }

    float metallicFactor = 0.0f;
    float roughnessFactor = 0.0f;

    // Check for metallic factor
    if (aiMat->Get(AI_MATKEY_METALLIC_FACTOR, metallicFactor) != aiReturn_SUCCESS) {
        metallicFactor = 0.0f; // Default if not found
    }
    // Check for roughness factor
    if (aiMat->Get(AI_MATKEY_ROUGHNESS_FACTOR, roughnessFactor) != aiReturn_SUCCESS) {
        roughnessFactor = 1.0f; // Default if not found
    }
    // Logic to determine the material type based on the PBR parameters
    if (metallicFactor < 0.1f && roughnessFactor > 0.5f) {
        material.type = 0;
    }
    else if (metallicFactor > 0.5f && roughnessFactor < 0.5f) {
        material.type = 1;
    }
    else if (metallicFactor > 0.0f && roughnessFactor > 0.5f) {
        material.type = 2;
    }
    // Default to diffuse if none of the above conditions match
    material.type = 0;


    // �������أ�����еĻ���
    aiString texturePath;
    if (aiMat->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath) == AI_SUCCESS) {
        GLuint texture = loadTextureFromFile(texturePath.C_Str());
        textures.push_back(texture);
    }

    return material;
}

GLuint Model::loadTextureFromFile(const std::string& texturePath) {
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // ʹ��STBͼƬ��������������������滻Ϊ����ͼƬ���ؿ⣩
    int width, height, nrChannels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
    if (data) {
        GLenum format = (nrChannels == 1) ? GL_RED : (nrChannels == 3) ? GL_RGB : GL_RGBA;
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        stbi_image_free(data);
    }
    else {
        std::cerr << "Failed to load texture: " << texturePath << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

void Model::draw() const {
    // ������Ե��Ի��ƴ��룬����ÿ������ʹ�����Ĳ��ʻ���
}