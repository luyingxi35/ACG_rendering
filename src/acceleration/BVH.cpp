#include "BVH.h"

AABB boundingBox(Triangle triangle) {
	glm::vec3 v0 = triangle.v0;
	glm::vec3 v1 = triangle.v1;
	glm::vec3 v2 = triangle.v2;
	glm::vec3 min = { std::min({v0.x, v1.x, v2.x}), std::min({v0.y, v1.y, v2.y}), std::min({v0.z, v1.z, v2.z}) };
	glm::vec3 max = { std::max({v0.x, v1.x, v2.x}), std::max({v0.y, v1.y, v2.y}), std::max({v0.z, v1.z, v2.z}) };
	return { min, max };
}

AABB BVH::computeBounds(const std::vector<Triangle>& triangles) {
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

BVHTreeNode* BVH::buildTree(std::vector<Triangle> triangles, int depth) {
	if (triangles.empty()) return nullptr;
	//std::cout << "triangles size: " << triangles.size() << std::endl;

	BVHTreeNode* node = new BVHTreeNode();
	num_nodes += 1;
	node->bounds = computeBounds(triangles);
	//std::cout << "Model min bound: " << node->bounds.min[0] << " " << node->bounds.min[1] << " " << node->bounds.min[2] << std::endl;
	//std::cout << "Model max bound: " << node->bounds.max[0] << " " << node->bounds.max[1] << " " << node->bounds.max[2] << std::endl;


	if (triangles.size() <= 50) {
		//std::cout << "Too depth or too small." << std::endl;
		node->triangles = triangles;
		assert(triangles.size() != 0);
		return node;
	}
	glm::vec3 extent = node->bounds.max - node->bounds.min;
	node->splitAxis = 0;
	if (extent[1] > extent[0])
		node->splitAxis = 1;
	if (extent[2] > extent[node->splitAxis])
		node->splitAxis = 2;
	
	int axis = node->splitAxis;
	//std::cout << "triangels szie " << triangles.size() << std::endl;
	std::sort(triangles.begin(), triangles.end(),
		[axis](const Triangle& a, const Triangle& b) -> bool {
			return a.centroid[axis] < b.centroid[axis];
		});
	//std::cout << "Triangles size: " << node->triangles.size() << std::endl;
	size_t mid = triangles.size() / 2;
	std::vector<Triangle> leftModels(triangles.begin(), triangles.begin() + mid);
	//std::cout << "begin - modelbegin:" << triangles.begin() - leftModels.begin() << std::endl;
	std::vector<Triangle> rightModels(triangles.begin() + mid, triangles.end());

	node->left = buildTree(leftModels, depth + 1);
	node->right = buildTree(rightModels, depth + 1);
	//std::cout << "depth: " << depth << std::endl;
	//std::cout << "Triangle size: " << node->triangles.size() << std::endl;
	return node;
}

int BVH::flattenTree(BVHTreeNode* node) {
	BVHFlatNode bvh_node{
		node->bounds,
		0,
		static_cast<int>(node->triangles.size()),
		node->splitAxis,
	};
	auto idx = nodes.size();
	nodes.push_back(bvh_node);
	if (bvh_node.tri_count == 0) {
		flattenTree(node->left);
		nodes[idx].child1_index = flattenTree(node->right);
	}
	else {
		nodes[idx].triangle_index = ordered_triangles.size();
		for (const auto& triangle : node->triangles) {
			ordered_triangles.push_back(triangle);
		}
	}
	return idx;
}

bool BVH::intersect(Ray& ray, Intersection& intersection, float tMin, float tMax) {
	/*if (!node || !node->bounds.intersect(ray, tMin, tMax)) {
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
	return hitLeft || hitRight;*/

	glm::bvec3 dir_is_neg = {
		ray.direction.x < 0,
		ray.direction.y < 0,
		ray.direction.z < 0,
	};

	std::array<int, 32> stack;
	auto ptr = stack.begin();
	int current_node_idx = 0;

	bool hit = false;

	while (true) {
		auto& node = nodes[current_node_idx];
		if (!node.bounds.intersect(ray, tMin, tMax)) {
			if (ptr == stack.begin()) {
				break;
			}
			current_node_idx = *(--ptr);
			continue;
		}
		if (node.tri_count == 0) {
			if (dir_is_neg[node.split_axis]) {
				*(ptr++) = current_node_idx + 1;
				current_node_idx = node.child1_index;
			}
			else {
				current_node_idx++;
				*(ptr++) = node.child1_index;
			}
		}
		else {
			float tTri = 1e7;
			glm::vec3 normal = { 0.0,0.0,0.0 };
			Material material = Material();
			auto triangle_iter = ordered_triangles.begin() + node.triangle_index;
			for (int i = 0; i < node.tri_count; i++) {
				if (triangle_iter->intersect(ray, tTri, normal) && tTri > tMin && tTri < tMax) {
					tMax = tTri;
					hit = true;
					material = triangle_iter->material;
					intersection.set(tTri, ray.position + tTri * ray.direction, normal, material);
				}
				triangle_iter++;
			}
			if (ptr == stack.begin()) {
				break;
			}
			current_node_idx = *(--ptr);
		}
	}
	/*if (hit) {
		std::cout << "hit\n";
	}
	else {
		std::cout << "not hit\n";
	}*/

	return hit;
}
