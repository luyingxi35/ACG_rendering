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
	Color tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount);

private:
	Color computeDiffuseLighting(Intersection& intersection, const Scene& scene);
	Color computeSpecularLighting(Intersection& intersection, const Scene& scene);
	glm::vec3 generateRandomDirection(glm::vec3 normal);
};

#endif