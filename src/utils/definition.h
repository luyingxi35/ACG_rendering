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

// ��Դ��
struct Light {
    glm::vec3 position;
    glm::vec3 u;
    glm::vec3 v;
    glm::vec3 color;
    float intensity;
    int samples;             // ÿ�����Դ�Ĳ�����������������Ӱ
};

// �����
struct Camera {
    glm::vec3 position;
    glm::mat3 rotationMatrix;
    float fov;
};

// ������
struct Ray {
    glm::vec3 position;
    glm::vec3 direction;
};

class Triangle {
public:
    glm::vec3 v0, v1, v2; // ��������

    
    bool intersect(const Ray& ray, float& t, glm::vec3& normal) {
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
            glm::vec3 position = rayOrigin + criteria * rayDir;
            normal = n;
            return true;
        }
        else {
            return false;
        }
    }

    // ���캯��
    Triangle(glm::vec3& v0, glm::vec3& v1, glm::vec3& v2)
        : v0(v0), v1(v1), v2(v2) {}
};
#endif
