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

struct AABB {
    glm::vec3 min, max;

    bool intersect(Ray& ray, float tMin, float tMax) {

        for (int i = 0; i < 3; ++i) {
            if (fabs(ray.direction[i]) < 1e-8f) { // 方向分量接近零
                if (ray.position[i] < min[i] || ray.position[i] > max[i]) {
                    return false; // parallel
                }
            }
            float invD = 1.0f / ray.direction[i];
            float t0 = (min[i] - ray.position[i]) * invD;
            float t1 = (max[i] - ray.position[i]) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            //std::cout << "t0: " << t0 << std::endl;
            //std::cout << "t1: " << t1 << std::endl;
            tMin = std::max(tMin, t0);
            tMax = std::min(tMax, t1);
            //std::cout << "tMin: " << tMin << std::endl;
            //std::cout << "tMax: " << tMax << std::endl;
            if (tMax <= tMin) return false;
        }
        return true;
    }

    void expand(const AABB& aabb) {
        min = glm::min(min, aabb.min);
        max = glm::max(max, aabb.max);
    }

    float surfaceArea() const {
        glm::vec3 d = glm::abs(max - min);
        return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    AABB() {
		min = glm::vec3(std::numeric_limits<float>::max());
		max = glm::vec3(std::numeric_limits<float>::lowest());
    }

	AABB(glm::vec3 min_, glm::vec3 max_) : min(min_), max(max_) {}
};

class Triangle {
public:
    glm::vec3 v0, v1, v2; // 三个顶点
    Material material;
    glm::vec3 centroid;
    AABB bounding_box;
    
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
            glm::vec3 position = rayOrigin + t * rayDir;
            normal = n;
            return true;
        }
        else {
            return false;
        }
    }

    // 构造函数
    Triangle(glm::vec3& v0, glm::vec3& v1, glm::vec3& v2, glm::vec3 vc, Material material)
        : v0(v0), v1(v1), v2(v2), centroid(vc), material(material), bounding_box(AABB(glm::min(v0,glm::min(v1, v2)), glm::max(v0, glm::max(v1, v2)))) {}
};
#endif
