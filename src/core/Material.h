#ifndef MATERIAL_H
#define MATERIAL_H
#include <glm/glm.hpp>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include "./Texture.h"

// Material types
enum class MaterialType {
	Diffuse,
	Plastic,
	Conductor,
	RoughDielectric,
	RoughConductor,
	RoughPlastic
};

class Material {
public:
	Material() {
		this->type = MaterialType::Diffuse;
		this->twoSided = false;
		this->diffuseReflect = glm::vec3(0.0f);
		this->specularReflect = glm::vec3(0.0f);
		this->alpha = 0.0f;
		this->int_ior = 0.0f;
		this->ext_ior = 0.0f;
		this->eta = glm::vec3(0.0f);
		this->k = glm::vec3(0.0f);
		this->nonlinear = false;
		this->emission = glm::vec3(0.0f);
		this->IsTexture = false;
		this->texture = nullptr;
	};
	~Material() {};
	MaterialType type;
	bool twoSided;  // 是否双面材质
	glm::vec3 diffuseReflect;  // 漫反射颜色
	glm::vec3 specularReflect;  // 镜面反射率
	float alpha;  // roughness, has a ggx distribution of microfacet normals
	float int_ior;  // inside折射率
	float ext_ior;  // 折射率
	glm::vec3 eta;  //for conductor
	glm::vec3 k;  //for conductor
	bool nonlinear;  //是否非线性
	glm::vec3 emission;  // 发光颜色
	bool IsTexture;
	std::shared_ptr<MipmapTexture> texture; // 使用智能指针管理纹理资源
};

#endif // !MATERIAL_H