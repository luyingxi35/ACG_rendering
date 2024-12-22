#ifndef BVH_H
#define BVH_H
#include "../scene/Scene.h"
#include "../core/Intersection.h"
#include "../utils/Profile.h"
#include <assert.h>
#include <algorithm>
#include <array>



struct BVHTreeNode {
	AABB bounds;                 // AABB for the node
	std::vector<Triangle> triangles;   // models in the node
	BVHTreeNode* left = nullptr;     // left subtree
	BVHTreeNode* right = nullptr;    // right subtree
	int splitAxis = 0;              // split axis

	bool isLeaf = false;
};

struct BVHFlatNode {
	AABB bounds;
	union {
		int child1_index;
		int triangle_index;
	};
	int tri_count = 0;
	int split_axis = 0;
	bool isLeaf = false;
};

class BVH {
private:
	BVHTreeNode* root;
	std::vector<BVHFlatNode> nodes;
	int num_nodes = 0;
	int num_triangles = 0;
	std::vector<Triangle> ordered_triangles;
	BVHTreeNode* buildTree(std::vector<Triangle> triangles, int depth);
	int flattenTree(BVHTreeNode* node);
	AABB computeBounds(const std::vector<Triangle>& triangles);
	//bool intersectNode(int node_index, Ray& ray, Intersection& intersection, float& t, float& tMin, float& tMax);
	void destroy(BVHTreeNode* node) {
		if (!node) return;
		destroy(node->left);
		destroy(node->right);
		delete node;
	}
public:
	BVH(const Scene& scene) {
		PROFILE("Build BVH")
			num_triangles = scene.triangles.size();
		//std::vector<Triangle> triangles = scene.triangles;
		root = buildTree(scene.triangles, 0);
		std::cout << "num_nodes: " << num_nodes << std::endl;
		std::cout << "num_triangles: " << num_triangles << std::endl;
		std::cout << "Finish build the BVH tree." << std::endl;
		std::cout << "Root: " << root->bounds.min[0] << ", " << root->bounds.min[1] << ", " << root->bounds.min[2] << ", " << "   "
			<< root->bounds.max[0] << ", " << root->bounds.max[1] << ", " << root->bounds.max[2] << std::endl;

		nodes.reserve(num_nodes);
		ordered_triangles.reserve(num_triangles);
		flattenTree(root);
	}
	~BVH() { destroy(root); }

	//bool intersect(Ray& ray, Intersection& intersection, float& t) { float tMin = 0.0f; float tMax = t; return intersectNode(0, ray, intersection, t, tMin, tMax); }
	bool intersect(Ray& ray, Intersection& intersection, float tMin, float tMax, const std::vector<Sphere>& spheres);
};
#endif