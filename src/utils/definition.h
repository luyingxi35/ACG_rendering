#ifndef DEFINITION_H
#define DEFINITION_H
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <iostream>
#include <filesystem>
#include <memory>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <cmath>
#include <string>
#include <random>


#define EPSILON 1e-6

// 光源类
struct Light {
    glm::vec3 position;
    glm::vec3 u;
    glm::vec3 v;
    glm::vec3 color;
    float intensity;
    int samples;             // 每个面光源的采样数量，用于软阴影
};

// 相机类
struct Camera {
    glm::vec3 position;
    glm::mat3 rotationMatrix;
    float fov;
};

// 光线类
struct Ray {
    glm::vec3 position;
    glm::vec3 direction;
};

class Triangle {
public:
    glm::vec3 v0, v1, v2; // 三个顶点
	glm::vec2 vt0, vt1, vt2; // 三个纹理坐标
    Material material;
    glm::vec3 centroid;
    
    bool intersect(const Ray& ray, float& t, glm::vec3& normal, glm::vec2& uv, int& mipLevel) {
        glm::vec3 rayOrigin = ray.position;
        glm::vec3 rayDir = glm::normalize(ray.direction);
        glm::vec3 e1 = v1 - v0;
        glm::vec3 e2 = v2 - v0;
        glm::vec3 n = glm::normalize(glm::cross(e1, e2));

        float denominator = glm::dot(rayDir, n);
        if (denominator > -EPSILON && denominator < EPSILON)
            return false;

        float criteria = dot(v0 - rayOrigin, n) / denominator;
        if (criteria < 0.0)
            return false;

        glm::vec3 s = rayOrigin - v0;
        glm::vec3 s1 = glm::cross(rayDir, e2);
        glm::vec3 s2 = glm::cross(s, e1);

        float invDenom = 1.0 / glm::dot(s1, e1);
        float b1 = dot(s1, s) * invDenom;
        float b2 = dot(s2, rayDir) * invDenom;
        t = dot(s2, e2) * invDenom;

        if (t > 1e-6 && b1 >= 0.0 && b2 >= 0.0 && b1 + b2 <= 1.0) {
            // 计算交点的 UV 坐标
            //float b3 = 1.0f - b1 - b2;  // 第三重心坐标

            //// 使用重心坐标计算 UV 坐标
            //uv = b1 * vt0 + b2 * vt1 + b3 * vt2;
            //if (uv.x != 0 && uv.y != 0) {
            //    // 计算纹理坐标梯度
            //    float dudx = abs(vt1.x - vt0.x), dvdx = abs(vt1.y - vt0.y);
            //    float dudy = abs(vt2.x - vt0.x), dvdy = abs(vt2.y - vt0.y);
            //    // 最大梯度决定采样率
            //    float maxScale = std::max(std::sqrt(dudx*dudx + dvdx*dvdx), std::sqrt(dudy*dudy + dvdy*dvdy));
            //    // 转换为 Mipmap 层索引（log2(maxScale)）
            //    mipLevel = std::max(0, static_cast<int>(std::round(std::log2(maxScale))));
                // 使用 round 保证 mipLevel 是整数

                /*std::cout << "e1 length: " << glm::length(e1) << ", e2 length: " << glm::length(e2) << std::endl;
                std::cout << "dudx: " << dudx << ", dvdx: " << dvdx << ", dudy: " << dudy << ", dvdy: " << dvdy << std::endl;
                std::cout << "maxScale: " << maxScale << std::endl;*/

                //std::cout << "mipLevel: " << mipLevel << std::endl;
            //}
            if (material.IsTexture) {
                int width = material.texture->widths[0], height = material.texture->heights[0];
                float texWidth = static_cast<float>(width);
                float texHeight = static_cast<float>(height);

                // 计算纹理坐标的分辨率尺度
                float textureSizeAtIntersection = glm::length(rayDir) * t;

                // 根据纹理投影尺度估算 Mipmap 层级
                float resolutionScale = textureSizeAtIntersection / std::max(texWidth, texHeight);

                // 通过对分辨率尺度取对数来选择合适的 Mipmap 层级
                int mipLevel = static_cast<int>(std::floor(std::log2(resolutionScale)));

                // 限制 Mipmap 层级的范围
                mipLevel = std::max(0, std::min(mipLevel, static_cast<int>(material.texture->mipLevels.size()) - 1));
            }

            glm::vec3 position = rayOrigin + t * rayDir;
            normal = n;
            return true;
        }
        else {
            return false;
        }
    }

    // 构造函数
    Triangle(glm::vec3& v0, glm::vec3& v1, glm::vec3& v2, glm::vec2& vt0, glm::vec2& vt1, glm::vec2& vt2, glm::vec3 vc, Material material)
        : v0(v0), v1(v1), v2(v2), vt0(vt0), vt1(vt1), vt2(vt2), centroid(vc), material(material){}
};
#endif
