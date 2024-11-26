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
    // ʵ���������ݣ���ĳ�ַ�ʽ��������
    unsigned char* data;
    int width, height;
};

#endif // TEXTURE_H