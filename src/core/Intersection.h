#ifndef INTERSECTION_H
#define INTERSECTION_H
#include "Material.h" // ��������һ��Material���ʾ����
#include "glm/glm.hpp"

class Intersection {
public:
    // Ĭ�Ϲ��캯��
    Intersection();

    // ���캯������ʼ�������������Ϣ
    Intersection(bool hit, float t, const glm::vec3& point, const glm::vec3& normal, Material material);

    // �Ƿ����˽���
    bool hit() const { return hit_; }

    float t() const { return t_; }

    // ��ȡ�����λ��
    const glm::vec3& point() const { return point_; }

    // ��ȡ����ķ���
    const glm::vec3& normal() const { return normal_; }

    // ��ȡ����Ĳ���
    Material& material() { return material_; }

    // ��ȡ�������������
    glm::vec2 uv() { return uv_; }

    // ���½�����Ϣ
    void set(const float t, const glm::vec3& point, const glm::vec3& normal, Material material, const glm::vec2& uv);

private:
    bool hit_;           // �Ƿ��н���
    float t_;
    glm::vec3 point_;      // ����λ��
    glm::vec3 normal_;     // ����ķ���
    glm::vec2 uv_;         // �������������
    Material material_; // ����Ĳ���
};

#endif // INTERSECTION_H

