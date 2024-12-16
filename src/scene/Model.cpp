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

//// 加载纹理并生成 Mipmap
//MipmapTexture Model::loadTextureAndGenerateMipmap(const std::string& texturePath) {
//	int width, height, nrChannels;
//	unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
//
//	if (!data) {
//		std::cerr << "Failed to load texture: " << texturePath << std::endl;
//		return {};
//	}
//
//	MipmapTexture mipmap;
//	mipmap.mipLevels.push_back(std::vector<unsigned char>(data, data + width * height * nrChannels));
//	mipmap.widths.push_back(width);
//	mipmap.heights.push_back(height);
//	mipmap.channels = nrChannels;
//
//	// 生成更低分辨率的 mipmap 层
//	while (width > 1 && height > 1) {
//		int newWidth = std::max(1, width / 2);
//		int newHeight = std::max(1, height / 2);
//		std::vector<unsigned char> newLevel(newWidth * newHeight * nrChannels);
//
//		for (int y = 0; y < newHeight; ++y) {
//			for (int x = 0; x < newWidth; ++x) {
//				for (int c = 0; c < nrChannels; ++c) {
//					int srcX = x * 2, srcY = y * 2;
//					unsigned char v1 = data[(srcY * width + srcX) * nrChannels + c];
//					unsigned char v2 = data[(srcY * width + srcX + 1) * nrChannels + c];
//					unsigned char v3 = data[((srcY + 1) * width + srcX) * nrChannels + c];
//					unsigned char v4 = data[((srcY + 1) * width + srcX + 1) * nrChannels + c];
//					newLevel[(y * newWidth + x) * nrChannels + c] = (v1 + v2 + v3 + v4) / 4;
//				}
//			}
//		}
//
//		mipmap.mipLevels.push_back(newLevel);
//		mipmap.widths.push_back(newWidth);
//		mipmap.heights.push_back(newHeight);
//		width = newWidth;
//		height = newHeight;
//	}
//
//	stbi_image_free(data);
//	return mipmap;
//}


//GLuint Model::loadTextureFromFile(const std::string& texturePath) {
//	GLuint textureID;
//	glGenTextures(1, &textureID); //生成纹理ID
//	glBindTexture(GL_TEXTURE_2D, textureID); //绑定纹理
//
//	// 使用STB图片库来加载纹理（你可以替换为其他图片加载库）
//	int width, height, nrChannels;
//	unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
//	if (data) {
//		GLenum format = (nrChannels == 1) ? GL_RED : (nrChannels == 3) ? GL_RGB : GL_RGBA;
//		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
//		////bilinear interpolation for filters
//		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//		//mipmap
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
//
