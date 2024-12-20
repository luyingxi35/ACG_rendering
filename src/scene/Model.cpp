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

	// 处理场景节点
	processNode(scene->mRootNode, scene);
	//std::cout << "Loading a model finished." << std::endl;
}

void Model::processNode(aiNode* node, const aiScene* scene) {
	// 处理当前节点的所有网格
	for (unsigned int i = 0; i < node->mNumMeshes; i++) {
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		processMesh(mesh, scene);
	}

	// 递归处理子节点
	for (unsigned int i = 0; i < node->mNumChildren; i++) {
		processNode(node->mChildren[i], scene);
	}
}

void Model::processMesh(aiMesh* mesh, const aiScene* scene) {
	ModelMesh modelMesh;

	// 获取顶点数据
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

	// 获取索引数据
	for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
		aiFace face = mesh->mFaces[i];
		for (unsigned int j = 0; j < face.mNumIndices; j++) {
			modelMesh.indices.push_back(face.mIndices[j]);
		}
	}

	meshes.push_back(modelMesh);
}

//GLuint Model::loadTextureFromFile(const std::string& texturePath) {
//	GLuint textureID;
//	glGenTextures(1, &textureID);
//	glBindTexture(GL_TEXTURE_2D, textureID);
//
//	// 使用STB图片库来加载纹理（你可以替换为其他图片加载库）
//	int width, height, nrChannels;
//	unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
//	if (data) {
//		GLenum format = (nrChannels == 1) ? GL_RED : (nrChannels == 3) ? GL_RGB : GL_RGBA;
//		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
//		glGenerateMipmap(GL_TEXTURE_2D);
//		stbi_image_free(data);
//	}
//	else {
//		std::cerr << "Failed to load texture: " << texturePath << std::endl;
//		stbi_image_free(data);
//	}
//
//	return textureID;
//}

