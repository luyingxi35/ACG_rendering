#pragma once

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <string>
#include <vector>

class AssimpLoader {
public:
    // 加载模型文件
    static const aiScene* loadModel(const std::string& path);
};
