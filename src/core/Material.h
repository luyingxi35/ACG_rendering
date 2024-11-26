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
    Material();
    ~Material();

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

