#pragma once

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <string>
#include <vector>

class AssimpLoader {
public:
    // ����ģ���ļ�
    static const aiScene* loadModel(const std::string& path);
};
