#include "BVH.h"

AABB boundingBox(Triangle triangle) {
	glm::vec3 v0 = triangle.v0;
	glm::vec3 v1 = triangle.v1;
	glm::vec3 v2 = triangle.v2;
	glm::vec3 min = { std::min({v0.x, v1.x, v2.x}), std::min({v0.y, v1.y, v2.y}), std::min({v0.z, v1.z, v2.z}) };
	glm::vec3 max = { std::max({v0.x, v1.x, v2.x}), std::max({v0.y, v1.y, v2.y}), std::max({v0.z, v1.z, v2.z}) };
	return { min, max };
}

AABB computeModelBounds(std::shared_ptr<Model> model) {
	if (model->triangles.empty()) {
		return AABB{ glm::vec3(FLT_MAX), glm::vec3(-FLT_MAX) }; // 返回一个空的 AABB
	}

	Triangle triangle_prime = model->triangles[0];
	AABB bounds = boundingBox(triangle_prime);
	for (auto& triangle : model->triangles) {
		glm::vec3 min = boundingBox(triangle).min;
		glm::vec3 max = boundingBox(triangle).max;
		//std::cout << "min" << min[0] << " " << min[1] << " " << min[2] << std::endl;
		//std::cout << "max" << max[0] << " " << max[1] << " " << max[2] << std::endl;
		bounds.min = { std::min(bounds.min.x, min.x), std::min(bounds.min.y, min.y), std::min(bounds.min.z, min.z) };
		bounds.max = { std::max(bounds.max.x, max.x), std::max(bounds.max.y, max.y), std::max(bounds.max.z, max.z) };
		//std::cout << "result" << bounds.min[0] << " " << bounds.min[1] << " " << bounds.min[2] << std::endl;
	}
	return bounds;
}

BVHNode* BVH::build(std::vector<std::shared_ptr<Model>> models, int depth) {
	if (models.empty()) return nullptr;

	BVHNode* node = new BVHNode();
	node->bounds = computeBounds(models);
	std::cout << "Model min bound: " << node->bounds.min[0] << " " << node->bounds.min[1] << " " << node->bounds.min[2] << std::endl;
	std::cout << "Model max bound: " << node->bounds.max[0] << " " << node->bounds.max[1] << " " << node->bounds.max[2] << std::endl;


	if (depth > 16 || models.size() <= 2) {
		std::cout << "Too depth or too small." << std::endl;
		node->models = std::move(models);
		return node;
	}
	int axis = depth % 3;
	std::sort(models.begin(), models.end(),
		[axis](std::shared_ptr<Model>& a, std::shared_ptr<Model>& b) {
			return computeModelBounds(a).min[axis] < computeModelBounds(b).min[axis];
		});

	size_t mid = models.size() / 2;
	std::vector< std::shared_ptr<Model>> leftModels(models.begin(), models.begin() + mid);
	std::vector< std::shared_ptr<Model>> rightModels(models.begin() + mid, models.end());

	node->left = build(leftModels, depth + 1);
	node->right = build(rightModels, depth + 1);
	std::cout << "depth: " << depth << std::endl;

	return node;
}
AABB BVH::computeBounds(std::vector<std::shared_ptr<Model>>& models) {
	AABB bounds = computeModelBounds(models[0]);
	for (auto& model : models) {
		AABB modelBounds = computeModelBounds(model);
		bounds.min = { std::min(bounds.min.x,modelBounds.min.x), std::min(bounds.min.y, modelBounds.min.y), std::min(bounds.min.z, modelBounds.min.z) };
		bounds.max = { std::max(bounds.max.x,modelBounds.max.x), std::max(bounds.max.y, modelBounds.max.y), std::max(bounds.max.z, modelBounds.max.z) };
	}
	return bounds;
}
bool BVH::intersectNode(BVHNode* node, Ray& ray, Intersection& intersection, float& t) {
	std::cout << t;
	float tMin = 0.0;
	float tMax = 1e6;
	if (!node || !node->bounds.intersect(ray, tMin, tMax)) {
		std::cout << "Not itersect with bounds." << std::endl;
		return false;
	}
	if (node->isLeaf()) {
		std::cout << "Leaf: ";
		bool hit = false;
		for (auto& model : node->models) {
			for (auto& triangle : model->triangles) {
				float tTri = 1e7;
				glm::vec3 normal = { 0.0,0.0,0.0 };
				if (triangle.intersect(ray, tTri, normal) && tTri < t) {
					t = tTri;
					hit = true;
					intersection.set(t, ray.position + t * ray.direction, normal, &model->material);
				}
			}
		}
		return hit;
	}
	bool hitLeft = intersectNode(node->left, ray, intersection, t);
	bool hitRight = intersectNode(node->right, ray, intersection, t);
	return hitLeft || hitRight;
}