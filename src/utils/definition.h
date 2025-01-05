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
#include "../core/Intersection.h"


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
public:
    glm::vec3 position;
    glm::vec3 direction;
    float time;
	glm::vec3 at(float t) {
		return position + t * direction;
	}
    glm::vec3 at(float t) const {
        return position + t * direction;
    }
	Ray(glm::vec3 origin, glm::vec3 direction, float time) : position(origin), direction(direction), time(time) {}
	Ray(glm::vec3 origin, glm::vec3 direction) : position(origin), direction(direction), time(0.0f) {}
};

struct AABB {
    glm::vec3 min, max;

    bool intersect(const Ray& ray, const glm::vec3& inv_direction, float t_min, float t_max) const {
        glm::vec3 t1 = (min - ray.position) * inv_direction;
        glm::vec3 t2 = (max - ray.position) * inv_direction;
        glm::vec3 tmin = glm::min(t1, t2);
        glm::vec3 tmax = glm::max(t1, t2);

        float near = glm::max(tmin.x, glm::max(tmin.y, tmin.z));
        float far = glm::min(tmax.x, glm::min(tmax.y, tmax.z));

        // 判断是否相交（提前退出以提高效率）
        if (near > far || far < t_min || near > t_max) {
            return false;
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
    
    bool intersect(const Ray& ray, float& t, glm::vec3& normal, float t_min, float t_max) {
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
		if (t_ > t_min && t_ < t_max) {
			normal = glm::normalize(glm::cross(e1, e2));
            t_max = t_;
            t = t_;
			return true;
		}
		return false;
    }

    // 构造函数
    Triangle(glm::vec3& v0, glm::vec3& v1, glm::vec3& v2, glm::vec3 vc, Material material)
        : v0(v0), v1(v1), v2(v2), centroid(vc), material(material), bounding_box(AABB(glm::min(v0,glm::min(v1, v2)), glm::max(v0, glm::max(v1, v2)))) {}
};

class Sphere {
public:
    Sphere(glm::vec3 center1, glm::vec3 center2, float radius, Material material) : center(center1, center1 - center2), radius(radius), material(material) {}
    Sphere(glm::vec3 center, float radius, Material material) : center(center, glm::vec3(0.0f)), radius(radius) , material(material){}
    bool intersect(const Ray& ray, float& t, glm::vec3& normal, float t_min, float t_max) {
        glm::vec3 current_center = center.at(ray.time);
		glm::vec3 co = ray.position - current_center;
		float a = glm::dot(ray.direction, ray.direction);
		float b = 2.0f * glm::dot(co, ray.direction);
		float c = glm::dot(co, co) - radius * radius;
		float discriminant = b * b - 4 * a * c;
        bool hit = false;
        if (discriminant < 0) {
			return false;
        }
		float t0 = (-b - sqrt(discriminant)) / (2.0f * a);
        if (t0 < 0) {
			t0 = (-b + sqrt(discriminant)) / (2.0f * a);
        }
        if (t0 > t_min && t0 < t_max) {
			normal = glm::normalize(ray.at(t0) - current_center);
			t = t0;
            hit = true;
        }
        return hit;
    }
    Material material;
	Ray center;
	float radius;
    
};

inline float random_float() {
    // Returns a random real in [0,1).
    return std::rand() / (RAND_MAX + 1.0);
}
inline float random_float(float min, float max) {
    // Returns a random real in [min,max).
    return min + (max - min) * random_float();
}
#endif
