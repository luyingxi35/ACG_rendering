#ifndef PATH_TRACER_H
#define PATH_TRACER_H
#define EPSILON 1e-6

#include <sstream>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <memory>
#include <glm/glm.hpp>
#include "../scene/Scene.h"
#include "Material.h"
#include "Color.h"
#include "Intersection.h"
#include "stb_image_write.h"
#include "../acceleration/BVH.h"
#include "../utils/definition.h"
#include <thread>
#include <vector>
#include <mutex>

class PathTracer {
public:
	void render(const Scene& scene, const Camera& camera, BVH& bvh, int width, int height, int samplesPerPixel, int numThreads);
	glm::vec3 tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount, std::mt19937& gen);
	void renderSection(const Scene& scene, const Camera& camera, BVH& bvh,
		int width, int height, int samplesPerPixel,
		int xStart, int xEnd,
		std::vector<glm::vec3>& framebuffer, std::mt19937& gen);

private:
	glm::vec3 computeDiffuseLighting(Intersection& intersection, BVH& bvh, const Scene& scene, std::mt19937& gen);
	glm::vec3 computeSpecularLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray);
	glm::vec3 refractDirection(const glm::vec3& incident, const glm::vec3& normal, float ext_ior, float int_ior);
	glm::vec3 computeRefractionLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray);
	glm::vec3 computeFresnelConductor(float cosTheta, const glm::vec3& eta, const glm::vec3& k);
	glm::vec3 generateRandomDirection(glm::vec3 normal, std::mt19937& gen);
};

#endif