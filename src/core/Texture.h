#ifndef TEXTURE_H
#define TEXTURE_H

#include "Color.h"
#include <string>
#include "stb_image.h"
#include <iostream>

class Texture {
public:
    Texture(const std::string& filepath);
    Color getColorAt(float u, float v) const;
private:
    // 实际纹理数据（以某种方式加载纹理）
    unsigned char* data;
    int width, height;
};

#endif // TEXTURE_H