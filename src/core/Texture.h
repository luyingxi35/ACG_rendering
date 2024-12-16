#ifndef TEXTURE_H
#define TEXTURE_H

#include "Color.h"
#include <string>
#include <vector>
#include "stb_image.h"
#include <iostream>

//class Texture {
//public:
//    Texture(const std::string& filepath);
//    Color getColorAt(float u, float v) const;
//private:
//    // 实际纹理数据（以某种方式加载纹理）
//    unsigned char* data;
//    int width, height;
//};

struct MipmapTexture {
    std::vector<std::vector<unsigned char>> mipLevels; // 每层的纹理数据
    std::vector<int> widths;                          // 每层纹理的宽度
    std::vector<int> heights;                         // 每层纹理的高度
    int channels;									 // 纹理通道数     
};

#endif // TEXTURE_H