#ifndef INTERSECTION_H
#define INTERSECTION_H
#include "Material.h" // 假设你有一个Material类表示材质
#include "glm/glm.hpp"

class Intersection {
public:
    // 默认构造函数
    Intersection();

    // 构造函数，初始化交点的所有信息
    Intersection(bool hit, float t, const glm::vec3& point, const glm::vec3& normal, Material material);

    // 是否发生了交点
    bool hit() const { return hit_; }

    float t() const { return t_; }

    // 获取交点的位置
    const glm::vec3& point() const { return point_; }

    // 获取交点的法线
    const glm::vec3& normal() const { return normal_; }

    // 获取交点的材质
    Material& material() { return material_; }

    // 获取交点的纹理坐标
    glm::vec2 uv() { return uv_; }

    // 更新交点信息
    void set(const float t, const glm::vec3& point, const glm::vec3& normal, Material material, const glm::vec2& uv);

private:
    bool hit_;           // 是否有交点
    float t_;
    glm::vec3 point_;      // 交点位置
    glm::vec3 normal_;     // 交点的法线
    glm::vec2 uv_;         // 交点的纹理坐标
    Material material_; // 交点的材质
};

#endif // INTERSECTION_H

