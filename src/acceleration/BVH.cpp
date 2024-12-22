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


	if (triangles.size() <= 50 || depth > 32) {
		//std::cout << "Too depth or too small." << std::endl;
		node->isLeaf = true;
		node->triangles = triangles;
		return node;
	}

	glm::vec3 diag = node->bounds.max - node->bounds.min;
	float min_cost = std::numeric_limits<float>::infinity();
	int min_split_index = 0;
	AABB min_left_bounds, min_right_bounds;
	int min_left_triangle_count = 0, min_right_triangle_count = 0;
	const int bucket_count = 12;
	std::vector<int> triangle_indices_bucket[3][bucket_count];

	for (int axis = 0; axis < 3; axis++) {
		AABB bounds_buckets[bucket_count] = {};
		int triangle_count_buckets[bucket_count] = {};
		int triangle_index = 0;
		for (const auto& triangle : triangles) {
			auto triangle_center = triangle.centroid[axis];
			int bucket_idx = glm::clamp<int>(glm::floor((triangle_center - node->bounds.min[axis]) * bucket_count / diag[axis]), 0, bucket_count - 1);
			bounds_buckets[bucket_idx].expand(triangle.bounding_box);
			triangle_count_buckets[bucket_idx]++;
			triangle_indices_bucket[axis][bucket_idx].push_back(triangle_index);
			triangle_index++;
		}

		AABB left_bounds = bounds_buckets[0];
		int left_triangle_count = triangle_count_buckets[0];
		for (int i = 1; i < bucket_count; i++) {
			AABB right_bounds;
			int right_triangle_count = 0;
			for (int j = bucket_count - 1; j >= i; j--) {
				right_bounds.expand(bounds_buckets[j]);
				right_triangle_count += triangle_count_buckets[j];
			}
			if (right_triangle_count == 0) {
				break;
			}
			if (left_triangle_count != 0) {
				float cost = left_triangle_count * left_bounds.surfaceArea() + right_triangle_count * right_bounds.surfaceArea();
				if (cost < min_cost) {
					min_cost = cost;
					node->splitAxis = axis;
					min_split_index = i;
					min_left_bounds = left_bounds;
					min_right_bounds = right_bounds;
					min_left_triangle_count = left_triangle_count;
					min_right_triangle_count = right_triangle_count;
				}
			}
			left_bounds.expand(bounds_buckets[i]);
			left_triangle_count += triangle_count_buckets[i];
		}
	}

	if (min_split_index == 0) {
		node->isLeaf = true;
		node->triangles = triangles;
		return node;
	}

	std::vector<Triangle> left_triangles, right_triangles;
	for (int i = 0; i < min_split_index; i++) {
		for (auto triangle_index : triangle_indices_bucket[node->splitAxis][i]) {
			left_triangles.push_back(triangles[triangle_index]);
		}
	}
	for (int i = min_split_index; i < bucket_count; i++) {
		for (auto triangle_index : triangle_indices_bucket[node->splitAxis][i]) {
			right_triangles.push_back(triangles[triangle_index]);
		}
	}

	node->left = buildTree(left_triangles, depth + 1);
	node->right = buildTree(right_triangles, depth + 1);

	return node;
}

int BVH::flattenTree(BVHTreeNode* node) {
	BVHFlatNode bvh_node{
		node->bounds,
		0,
		static_cast<int>(node->triangles.size()),
		node->splitAxis,
		node->isLeaf,
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

bool BVH::intersect(Ray& ray, Intersection& intersection, float tMin, float tMax, const std::vector<Sphere> &spheres) {
	glm::bvec3 dir_is_neg = {
		ray.direction.x < 0,
		ray.direction.y < 0,
		ray.direction.z < 0,
	};
	glm::vec3 inv_dir = 1.0f / ray.direction;

	std::array<int, 32> stack;
	auto ptr = stack.begin();
	int current_node_idx = 0;

	bool hit = false;

	while (true) {
		auto& node = nodes[current_node_idx];
		if (!node.bounds.intersect(ray, inv_dir, tMin, tMax)) {
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
			glm::vec2 uv = { 0.0,0.0 };
			glm::vec3 point = { 0.0,0.0,0.0 };
			Material material = Material();
			auto triangle_iter = ordered_triangles.begin() + node.triangle_index;
			for (int i = 0; i < node.tri_count; i++) {
				if (triangle_iter->intersect(ray, tTri, normal, tMin, tMax, uv, point)) {
					tMax = tTri;
					hit = true;
					material = triangle_iter->material;
					intersection.set(tTri, point, normal, material, uv);
				}
				triangle_iter++;
			}
			if (ptr == stack.begin()) {
				break;
			}
			current_node_idx = *(--ptr);
		}
	}
	float tSph = tMax;
	glm::vec3 normal = { 0.0,0.0,0.0 };
	glm::vec2 uv = { 0.0,0.0 };
	for (auto sphere : spheres) {
		//std::cout << "Before sphere: " << tMax << std::endl;
		if (sphere.intersect(ray, tSph, normal, tMin, tMax) && tSph < tMax && tSph > tMin) {
			hit = true;
			tMax = tSph;
			intersection.set(tSph, ray.position + tSph * ray.direction, normal, sphere.material, uv);
			//std::cout << "Hit the sphere." << std::endl;
			//std::cout << "After sphere: " << tSph << std::endl;

		}
	}

	return hit;
}

