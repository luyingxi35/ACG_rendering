#ifndef DEFINITION_H
#define DEFINITION_H
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <iostream>
#include <filesystem>
#include <memory>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <cmath>
#include <string>
#include <random>


#define EPSILON 1e-6

// ��Դ��
struct Light {
    glm::vec3 position;
    glm::vec3 u;
    glm::vec3 v;
    glm::vec3 color;
    float intensity;
    int samples;             // ÿ�����Դ�Ĳ�����������������Ӱ
};

// �����
struct Camera {
    glm::vec3 position;
    glm::mat3 rotationMatrix;
    float fov;
};

// ������
struct Ray {
    glm::vec3 position;
    glm::vec3 direction;
};

struct AABB {
    glm::vec3 min, max;

    bool intersect(const Ray& ray, const glm::vec3& inv_direction, float t_min, float t_max) const {
        glm::vec3 t1 = (min - ray.position) * inv_direction;
        glm::vec3 t2 = (max - ray.position) * inv_direction;
        glm::vec3 tmin = glm::min(t1, t2);
        glm::vec3 tmax = glm::max(t1, t2);

        float near = glm::max(tmin.x, glm::max(tmin.y, tmin.z));
        float far = glm::min(tmax.x, glm::min(tmax.y, tmax.z));

        // �ж��Ƿ��ཻ����ǰ�˳������Ч�ʣ�
        if (near > far || far < t_min || near > t_max) {
            return false;
        }

        return true;
    }

    void expand(const AABB& aabb) {
        min = glm::min(min, aabb.min);
        max = glm::max(max, aabb.max);
    }

    float surfaceArea() const {
        glm::vec3 d = glm::abs(max - min);
        return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    AABB() {
		min = glm::vec3(std::numeric_limits<float>::max());
		max = glm::vec3(std::numeric_limits<float>::lowest());
    }

	AABB(glm::vec3 min_, glm::vec3 max_) : min(min_), max(max_) {}
};

class Triangle {
public:
    glm::vec3 v0, v1, v2; // ��������
    glm::vec2 vt0, vt1, vt2; // ������������
    Material material;
    glm::vec3 centroid;
    AABB bounding_box;\

        float bilinearInterpolate(const glm::vec2& uv, std::shared_ptr<Texture> texture) {
        int width = texture->width;
        int height = texture->height;

        // uv �ǹ�һ�����������꣬��Χ�� [0, 1] ֮��
        float x = uv.x * (width - 1);  // �� uv.x ӳ�䵽�����ˮƽ����
        float y = uv.y * (height - 1); // �� uv.y ӳ�䵽����Ĵ�ֱ����

        int x0 = static_cast<int>(x);   // ��������
        int y0 = static_cast<int>(y);   // �ײ������
        int x1 = std::min(x0 + 1, width - 1); // �Ҳ�����㣨��ֹԽ�磩
        int y1 = std::min(y0 + 1, height - 1); // ��������㣨��ֹԽ�磩

        float hValue0 = texture->heightData[y0 * width + x0];
        float hValue1 = texture->heightData[y0 * width + x1];
        float hValue2 = texture->heightData[y1 * width + x0];
        float hValue3 = texture->heightData[y1 * width + x1];

        // ˮƽ��ֵ (x����)
        float R1 = hValue0 + (x - x0) * (hValue2 - hValue1); // f(x, y0)
        float R2 = hValue2 + (x - x0) * (hValue3 - hValue2); // f(x, y1)

        //std::cout << "R1: " << R1 << " R2: " << R2 << std::endl;

        // ��ֱ��ֵ (y����)
        float heightValue = R1 + (y - y0) * (R2 - R1);

        // ʹ���ݶȼ��㷨��(local)
        glm::vec3 Heightnormal(hValue1 - hValue0, hValue3 - hValue2, 2.0f);  // 2.0f ���ڷŴ�Ч��
        Heightnormal = glm::normalize(Heightnormal);

        return heightValue;
    }

    glm::vec3 bilinearInterpolateNormal(const glm::vec2& uv, std::shared_ptr<Texture> texture) {
        int width = texture->width;
        int height = texture->height;

        // uv �ǹ�һ�����������꣬��Χ�� [0, 1] ֮��
        float x = uv.x * (width - 1);  // �� uv.x ӳ�䵽�����ˮƽ����
        float y = uv.y * (height - 1); // �� uv.y ӳ�䵽����Ĵ�ֱ����

        int x0 = static_cast<int>(x);   // ��������
        int y0 = static_cast<int>(y);   // �ײ������
        int x1 = std::min(x0 + 1, width - 1); // �Ҳ�����㣨��ֹԽ�磩
        int y1 = std::min(y0 + 1, height - 1); // ��������㣨��ֹԽ�磩

        float hValue0 = texture->heightData[y0 * width + x0];
        float hValue1 = texture->heightData[y0 * width + x1];
        float hValue2 = texture->heightData[y1 * width + x0];
        float hValue3 = texture->heightData[y1 * width + x1];

		//std::cout << "hValue0: " << hValue0 << " hValue1: " << hValue1 << " hValue2: " << hValue2 << " hValue3: " << hValue3 << std::endl;

        // ˮƽ��ֵ (x����)
        float R1 = hValue0 + (x - x0) * (hValue1 - hValue0); // f(x, y0)
        float R2 = hValue2 + (x - x0) * (hValue3 - hValue2); // f(x, y1)

        //std::cout << "R1: " << R1 << " R2: " << R2 << std::endl;

        // ��ֱ��ֵ (y����)
        float heightValue = R1 + (y - y0) * (R2 - R1);

        // ʹ���ݶȼ��㷨��(local)
        glm::vec3 Heightnormal(hValue1 - hValue0, hValue3 - hValue2, 0.1f);  // 0.1f ���ڷŴ�Ч��
        Heightnormal = glm::normalize(Heightnormal);

        return Heightnormal;
    }
    
    bool intersect(const Ray& ray, float& t, glm::vec3& normal, float t_min, float t_max, glm::vec2& uv, glm::vec3& point) {
        glm::vec3 e1 = v1 - v0;
		glm::vec3 e2 = v2 - v0;
		glm::vec3 h = glm::cross(ray.direction, e2);
		float a = glm::dot(e1, h);

		if (a > -EPSILON && a < EPSILON) {
			return false;
		}

        float f = 1.0f / a;
		glm::vec3 s = ray.position - v0;
        float u = f * glm::dot(s, h);
        if (u < 0.0 || u > 1.0)
            return false;

		glm::vec3 q = glm::cross(s, e1);
		float v = f * glm::dot(ray.direction, q);
        if (v < 0.0 || u + v > 1.0)
            return false;

		float t_ = f * glm::dot(e2, q);
        normal = glm::normalize(glm::cross(e1, e2));
        //// change normal to color for debugging
        //glm::vec3 normalColor = (normal * 0.5f) + glm::vec3(0.5f, 0.5f, 0.5f);
        //normal = normalColor;
        
		point = ray.position + t_ * ray.direction;
		if (t_ > t_min && t_ < t_max) {
            //normal = glm::normalize(glm::cross(e1, e2));
            if (material.IsTexture) {
                //���㽻��� UV ����
                float b3 = 1.0f - u - v;  // ������������

                //ʹ������������� UV ����
                uv = b3 * vt0 + u * vt1 + v * vt2;

                if (material.IsTextureHeight) {
					float heightValue = bilinearInterpolate(uv, material.texture);
					//std::cout << "heightValue: " << heightValue << std::endl;
                    
                    // �� Z �᷽����ƫ�ƶ����λ�ã�Z���ʾ�߶ȣ�
                    point.y += heightValue * 5.0f;  // 10.0f: �������ӣ����ڷŴ�Ч��
					//std::cout << "point_z: " << point.z << std::endl;

					glm::vec3 Heightnormal = bilinearInterpolateNormal(uv, material.texture);

					//std::cout << "Heightnormal: " << Heightnormal[0] << " " << Heightnormal[1] << " " << Heightnormal[2] << std::endl;

					//TBN, transfrom local normal to world normal
                    glm::vec2 deltauv1 = vt1 - vt0;
                    glm::vec2 deltauv2 = vt2 - vt0;
					glm::mat3 TBN = glm::mat3(1.0f);

                    float determinant = deltauv1.x * deltauv2.y - deltauv1.y * deltauv2.x;
                    if (std::abs(determinant) > 1e-6f) {
                        float r = 1.0f / determinant;

                        glm::vec3 tangent = glm::normalize((deltauv2.y * e1 - deltauv1.y * e2) * r);
                        glm::vec3 bitangent = glm::normalize((deltauv1.x * e2 - deltauv2.x * e1) * r);

                        glm::vec3 rebitangent = glm::cross(normal, tangent);
                        if (glm::dot(rebitangent, bitangent) < 0.0f) {
                            bitangent = -bitangent;
                        }
                        TBN = glm::mat3(tangent, bitangent, normal);
                    }

					//�����ߴӾֲ��ռ�ת��������ռ�
					glm::vec3 worldHeightNormal = glm::normalize(TBN * Heightnormal);

                    normal = worldHeightNormal;
					normal = glm::normalize(normal);
					//std::cout << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;
                }
            }
            t_max = t_;
            t = t_;
			return true;
		}
		return false;
    }

    // ���캯��
    Triangle(glm::vec3& v0, glm::vec3& v1, glm::vec3& v2, glm::vec2& vt0, glm::vec2& vt1, glm::vec2& vt2, glm::vec3 vc, Material material)
        : v0(v0), v1(v1), v2(v2), vt0(vt0), vt1(vt1), vt2(vt2), centroid(vc), material(material), bounding_box(AABB(glm::min(v0,glm::min(v1, v2)), glm::max(v0, glm::max(v1, v2)))) {}
};
#endif
