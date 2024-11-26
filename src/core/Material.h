#ifndef MATERIAL_H
#define MATERIAL_H
#include <glm/glm.hpp>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
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
	};
	~Material() {};
	MaterialType type;
	bool twoSided;  // �Ƿ�˫�����
	glm::vec3 diffuseReflect;  // ��������ɫ
	glm::vec3 specularReflect;  // ���淴����
	float alpha;  // roughness, has a ggx distribution of microfacet normals
	float int_ior;  // inside������
	float ext_ior;  // ������
	glm::vec3 eta;  //for conductor
	glm::vec3 k;  //for conductor
	bool nonlinear;  //�Ƿ������
};

#endif // !MATERIAL_H