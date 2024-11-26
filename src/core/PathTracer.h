#ifndef PATH_TRACER_H
#define PATH_TRACER_H
#define EPSILON 1e-6

#include <sstream>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <memory>
#include "../scene/Scene.h"
#include "Material.h"
#include "Color.h"
#include "Intersection.h"
#include "stb_image_write.h"
#include "../acceleration/BVH.h"
#include "../utils/definition.h"

class PathTracer {
public:
	void render(const Scene& scene, const Camera& camera, BVH& bvh, int width, int height, int samplesPerPixel);
	glm::vec3 tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount);

private:
	glm::vec3 computeDiffuseLighting(Intersection& intersection, BVH& bvh, const Scene& scene);
	glm::vec3 computeSpecularLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray);
	glm::vec3 refractDirection(const glm::vec3& incident, const glm::vec3& normal, float ext_ior, float int_ior);
	glm::vec3 computeRefractionLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray);
	glm::vec3 computeFresnelConductor(float cosTheta, const glm::vec3& eta, const glm::vec3& k);
	glm::vec3 generateRandomDirection(glm::vec3 normal);
};

#endif