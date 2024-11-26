#ifndef PATH_TRACER_H
#define PATH_TRACER_H

#include "../scene/Scene.h"
#include "Material.h"
#include "Color.h"
#include "Intersection.h"
#include <vector>
#include <glm/glm.hpp>
#include <fstream>
#include "../scene/Model.h"
#include <cmath>
#include <random>
#include <iostream>
#include "stb_image_write.h"

class Triangle {
public:
	glm::vec3 v0, v1, v2; // 三个顶点

	// 构造函数
	Triangle(glm::vec3& v0, glm::vec3& v1, glm::vec3& v2)
		: v0(v0), v1(v1), v2(v2) {}
};

class PathTracer {
public:
	void render(const Scene& scene, const Camera& camera, int width, int height, int samplesPerPixel);
	glm::vec3 tracePath(Ray ray, const Scene& scene, int bounceCount);

private:
	glm::vec3 computeDiffuseLighting(Intersection& intersection, const Scene& scene);
	glm::vec3 computeSpecularLighting(Intersection& intersection, const Scene& scene, const Ray& ray);
	glm::vec3 refractDirection(const glm::vec3& incident, const glm::vec3& normal, float ext_ior, float int_ior);
	glm::vec3 computeRefractionLighting(Intersection& intersection, const Scene& scene, const Ray& ray);
	glm::vec3 generateRandomDirection(glm::vec3 normal);
	glm::vec3 computeFresnelConductor(float cosTheta, const glm::vec3& eta, const glm::vec3& k);
};

#endif