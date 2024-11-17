#include "Intersection.h"

// Ĭ�Ϲ��캯������ʾû�н���
Intersection::Intersection()
    : hit_(false), t_(0), point_(glm::vec3(0.0f)), normal_(glm::vec3(0.0f)), material_(nullptr) {}

// ���캯������ʼ�������������Ϣ
Intersection::Intersection(bool hit, float t, const glm::vec3& point, const glm::vec3& normal, Material* material)
    : hit_(hit), t_(t), point_(point), normal_(normal), material_(material) {}

// ���ý�����Ϣ
void Intersection::set(const float t, const glm::vec3& point, const glm::vec3& normal, Material* material) {
    hit_ = true;
    t_ = t;
    point_ = point;
    normal_ = normal;
    material_ = material;
}
