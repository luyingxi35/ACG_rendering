#ifndef TEXTURE_H
#define TEXTURE_H

#include "Color.h"
#include <string>
#include "stb_image.h"
#include <iostream>

class Texture {
public:
    unsigned char* data;  // 存储纹理的图像数据
    int width, height;    // 纹理的宽度和高度

    // 析构函数来释放纹理数据
    ~Texture() {
        if (data) {
            stbi_image_free(data);
        }
    }
};
#endif // TEXTURE_H