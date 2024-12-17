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

        glm::vec3 t1 = (min - ray.position) / ray.direction;
        glm::vec3 t2 = (max - ray.position) / ray.direction;
        glm::vec3 tmin = glm::min(t1, t2);
        glm::vec3 tmax = glm::max(t1, t2);

        float near = glm::max(tmin.x, glm::max(tmin.y, tmin.z));
        float far = glm::min(tmax.x, glm::min(tmax.y, tmax.z));

        if (near <= tMin && far >= tMax) {
            return false;
        }

        return glm::max(near, tMin) <= glm::min(far, tMax);
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
        glm::vec3 e1 = v1 - v0;
		glm::vec3 e2 = v2 - v0;
		glm::vec3 h = glm::cross(ray.direction, e2);
		float a = glm::dot(e1, h);

		if (a > -EPSILON && a < EPSILON) {
			return false;
		}

        float f = 1.0f / a;
		glm::vec3 s = ray.position - v0;
        float u = f * glm::dot(s, h);
        if (u < 0.0 || u > 1.0)
            return false;

		glm::vec3 q = glm::cross(s, e1);
		float v = f * glm::dot(ray.direction, q);
        if (v < 0.0 || u + v > 1.0)
            return false;

		float t_ = f * glm::dot(e2, q);
		if (t_ > EPSILON) {
			normal = glm::normalize(glm::cross(e1, e2));
            t = t_;
			return true;
		}
		return false;
    }

    // 构造函数
    Triangle(glm::vec3& v0, glm::vec3& v1, glm::vec3& v2, glm::vec3 vc, Material material)
        : v0(v0), v1(v1), v2(v2), centroid(vc), material(material), bounding_box(AABB(glm::min(v0,glm::min(v1, v2)), glm::max(v0, glm::max(v1, v2)))) {}
};
#endif
