#include "Intersection.h"

// 默认构造函数，表示没有交点
Intersection::Intersection()
    : hit_(false), t_(0), point_(glm::vec3(0.0f)), normal_(glm::vec3(0.0f)), material_(Material()) {}

// 构造函数，初始化交点的所有信息
Intersection::Intersection(bool hit, float t, const glm::vec3& point, const glm::vec3& normal, Material material)
    : hit_(hit), t_(t), point_(point), normal_(normal), material_(material) {}

// 设置交点信息
void Intersection::set(const float t, const glm::vec3& point, const glm::vec3& normal, Material material, const glm::vec2& uv, int& mipLevel) {
    hit_ = true;
    t_ = t;
    point_ = point;
    normal_ = normal;
    uv_ = uv;
    material_ = material;
    mipLevel_ = mipLevel;
}
