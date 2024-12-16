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
//    // ʵ���������ݣ���ĳ�ַ�ʽ��������
//    unsigned char* data;
//    int width, height;
//};

struct MipmapTexture {
    std::vector<std::vector<unsigned char>> mipLevels; // ÿ�����������
    std::vector<int> widths;                          // ÿ������Ŀ��
    std::vector<int> heights;                         // ÿ������ĸ߶�
    int channels;									 // ����ͨ����     
};

#endif // TEXTURE_H