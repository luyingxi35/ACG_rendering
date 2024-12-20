#ifndef TEXTURE_H
#define TEXTURE_H

#include "Color.h"
#include <string>
#include "stb_image.h"
#include <iostream>

class Texture {
public:
    unsigned char* data;  // �洢�����ͼ������
    int width, height;    // ����Ŀ�Ⱥ͸߶�

    // �����������ͷ���������
    ~Texture() {
        if (data) {
            stbi_image_free(data);
        }
    }
};
#endif // TEXTURE_H