#ifndef BVH_H
#define BVH_H
#include "../scene/Scene.h"
#include "../core/Intersection.h"
#include "../utils/Profile.h"
#include <assert.h>
#include <algorithm>

struct AABB {
	glm::vec3 min, max;

	bool intersect(Ray& ray, float& tMin, float& tMax) {

		for (int i = 0; i < 3; ++i) {
			if (fabs(ray.direction[i]) < 1e-8f) { // 方向分量接近零
				if (ray.position[i] < min[i] || ray.position[i] > max[i]) {
					return false; // parallel
				}
			}
			float invD = 1.0 / ray.direction[i];
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
};

struct BVHNode {
	AABB bounds;                 // AABB for the node
	std::vector<Triangle> triangles;   // models in the node
	BVHNode* left = nullptr;     // left subtree
	BVHNode* right = nullptr;    // right subtree

	bool isLeaf() const {
		return left == nullptr && right == nullptr;
	}
};

class BVH {
private:
	BVHNode* root;
	BVHNode* build(std::vector<Triangle> triangles, int depth);
	AABB computeBounds(const std::vector<Triangle>& triangles);
	bool intersectNode(BVHNode* node, Ray& ray, Intersection& intersection, float& t);
	void destroy(BVHNode* node) {
		if (!node) return;
		destroy(node->left);
		destroy(node->right);
		delete node;
	}
public:
	BVH(const Scene& scene) {
		PROFILE("Build BVH tree")
		//std::vector<Triangle> triangles = scene.triangles;
 		root = build(scene.triangles, 0);
		std::cout << "Finish build the BVH tree." << std::endl;
		std::cout << "Root: " << root->bounds.min[0] << ", " << root->bounds.min[1] << ", " << root->bounds.min[0] << ", " << "   "
			<< root->bounds.max[0] << ", " << root->bounds.max[1] << ", " << root->bounds.max[2] << std::endl;
	}
	~BVH() { destroy(root); }

	bool intersect(Ray& ray, Intersection& intersection, float& t) { return intersectNode(root, ray, intersection, t); }
};
#endif



