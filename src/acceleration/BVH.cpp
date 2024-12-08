#include "BVH.h"

AABB boundingBox(Triangle triangle) {
	glm::vec3 v0 = triangle.v0;
	glm::vec3 v1 = triangle.v1;
	glm::vec3 v2 = triangle.v2;
	glm::vec3 min = { std::min({v0.x, v1.x, v2.x}), std::min({v0.y, v1.y, v2.y}), std::min({v0.z, v1.z, v2.z}) };
	glm::vec3 max = { std::max({v0.x, v1.x, v2.x}), std::max({v0.y, v1.y, v2.y}), std::max({v0.z, v1.z, v2.z}) };
	return { min, max };
}

AABB BVH::computeBounds(std::vector<Triangle>& triangles) {
	glm::vec3 min = glm::vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	glm::vec3 max = glm::vec3(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
	for (auto triangle : triangles) {
		glm::vec3 triangle_min = boundingBox(triangle).min;
		glm::vec3 triangle_max = boundingBox(triangle).max;
		min = { std::min(min.x, triangle_min.x), std::min(min.y, triangle_min.y), std::min(min.z, triangle_min.z) };
		max = { std::max(max.x, triangle_max.x), std::max(max.y, triangle_max.y), std::max(max.z, triangle_max.z) };
	}
	return { min, max };
}

BVHNode* BVH::build(std::vector<Triangle> triangles, int depth) {
	if (triangles.empty()) return nullptr;

	BVHNode* node = new BVHNode();
	node->bounds = computeBounds(triangles);
	//std::cout << "Model min bound: " << node->bounds.min[0] << " " << node->bounds.min[1] << " " << node->bounds.min[2] << std::endl;
	//std::cout << "Model max bound: " << node->bounds.max[0] << " " << node->bounds.max[1] << " " << node->bounds.max[2] << std::endl;


	if (triangles.size() <= 16) {
		//std::cout << "Too depth or too small." << std::endl;
		node->triangles = std::move(triangles);
		return node;
	}
	glm::vec3 extent = node->bounds.max - node->bounds.min;
	int axis = 0;
	if (extent[1] > extent[0])
		axis = 1;
	if (extent[2] > extent[axis])
		axis = 2;
	std::sort(triangles.begin(), triangles.end(),
		[axis](const Triangle& a, const Triangle& b) -> bool {
			return a.centroid[axis] < b.centroid[axis];
		});

	size_t mid = triangles.size() / 2;
	std::vector< Triangle> leftModels(triangles.begin(), triangles.begin() + mid);
	std::vector< Triangle> rightModels(triangles.begin() + mid, triangles.end());

	node->left = build(leftModels, depth + 1);
	node->right = build(rightModels, depth + 1);
	//std::cout << "depth: " << depth << std::endl;

	return node;
}
bool BVH::intersectNode(BVHNode* node, Ray& ray, Intersection& intersection, float& t) {
	//std::cout << t;
	float tMin = 0.0;
	float tMax = t;
	if (!node || !node->bounds.intersect(ray, tMin, tMax)) {
		//std::cout << "Not itersect with bounds." << std::endl;
		return false;
	}
	if (node->isLeaf()) {
		//std::cout << "Leaf: ";
		bool hit = false;
		Material material = Material();
		for (auto& triangle : node->triangles) {
			float tTri = 1e7;
			glm::vec3 normal = { 0.0,0.0,0.0 };
			if (triangle.intersect(ray, tTri, normal) && tTri > tMin && tTri < t) {
				t = tTri;
				hit = true;
				material = triangle.material;
				intersection.set(t, ray.position + t * ray.direction, normal, material);
			}
		}
		return hit;
	}
	bool hitLeft = intersectNode(node->left, ray, intersection, t);
	bool hitRight = intersectNode(node->right, ray, intersection, t);
	return hitLeft || hitRight;
}