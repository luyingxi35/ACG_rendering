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
#include <cmath>
#include <queue>
#include "../utils/Profile.h"
#include "../scene/hdr.h"

struct Task {
	int xStart;
	int xEnd;
	int yStart;
	int yEnd;
};

class PathTracer {
public:
	void render(const Scene& scene, const Camera& camera, BVH& bvh, int width, int height, int samplesPerPixel, int numThreads);
	glm::vec3 tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount, std::mt19937& gen);
	/*void renderSection(const Scene& scene, const Camera& camera, BVH& bvh,
		int width, int height, int samplesPerPixel,
		int xStart, int xEnd,
		std::vector<glm::vec3>& framebuffer, std::mt19937& gen);*/
	void renderWorker(const Scene& scene, const Camera& camera, BVH& bvh, int width, int height, int samplesPerPixel,
		std::vector<glm::vec3>& framebuffer, std::mt19937& gen);
	void enqueueTasks(int xtileSize, int ytileSize, int width, int height) {
		for (int y = 0; y < height; y += ytileSize) {
			for (int x = 0; x < width; x += xtileSize) {
				int xEnd = std::min(x + xtileSize, width);
				int yEnd = std::min(y + ytileSize, height);
				Task task = { x, xEnd, y, yEnd };
				{
					std::lock_guard < std::mutex > lock(queue_mutex);
					task_queue.emplace(task);
				}
				condition.notify_one();
				//{
				//	std::lock_guard<std::mutex> lock(cout_mutex);
					//std::cout << "Finish adding task (" << x << ", " << y << ")\n";
				//}
			}
		}
		{
			std::lock_guard<std::mutex> lock(queue_mutex);
			done = true;
		}
		condition.notify_all();
	}
	void loadEnvironmentMap(const std::string& hdrFilePath) {
		try {
			environmentMap = loadHDR(hdrFilePath); // 调用 HDR 加载函数
			useHDR = true;
			std::cout << "HDR environment map loaded: " << hdrFilePath << std::endl;
		}
		catch (const std::exception& e) {
			std::cerr << "Failed to load HDR map: " << e.what() << std::endl;
			useHDR = false;
		}
	}

private:
	HDRImage environmentMap; // 存储加载的 HDR 纹理
	bool useHDR = false;     // 标志是否启用 HDR
	std::mutex queue_mutex;
	std::condition_variable condition;
	std::queue<Task> task_queue;
	bool done = false;
	std::mutex cout_mutex;

	glm::vec3 computeDiffuseLighting(Intersection& intersection, BVH& bvh, const Scene& scene, std::mt19937& gen, float &light_pdf);
	glm::vec3 computeSpecularLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray);
	glm::vec3 refractDirection(const glm::vec3& incident, const glm::vec3& normal, float ext_ior, float int_ior);
	glm::vec3 computeRefractionLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray);
	glm::vec3 computeFresnelConductor(float cosTheta, const glm::vec3& eta, const glm::vec3& k);
	glm::vec3 generateRandomDirection(glm::vec3 normal, std::mt19937& gen);

};

#endif