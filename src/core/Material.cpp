#include "Material.h"

Material::Material(){
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
}
Material::~Material() {}


