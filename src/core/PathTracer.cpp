﻿#include "PathTracer.h"
#define EPSILON 1e-6
#define M_PI 3.1415926535

const int MAX_BOUNCES = 16;
const float P = 0.6666;

//point light
/*bool intersectLight(const Ray& ray, const std::vector<Light> lights, Light& result_light, Intersection& intersection) {
    bool hit = false;
    glm::vec3 pos = ray.position;
    glm::vec3 dir = glm::normalize(ray.direction);
    Material material = Material();
    Material* p = &material;
    float closestHit = std::numeric_limits<float>::max();
    for (const auto& light : lights) {
        float t = glm::length(light.position - pos);
        if (glm::dot(light.position - pos, dir) > 0 && (1 - glm::dot(light.position - pos, dir)) < EPSILON && t < closestHit) {
            hit = true;
            closestHit = t;
            result_light = light;
            intersection.set(t, glm::vec3(0.0f), glm::vec3(0.0f), p);
        }
    }
    return hit;
}*/

glm::vec3 sampleAreaLight(const Light& light, int sampleIndex, int totalSamples) {
    // 假设 totalSamples 是平方数，例如16 = 4x4网格
    int sqrtSamples = static_cast<int>(std::sqrt(static_cast<float>(totalSamples)));
    int x = sampleIndex % sqrtSamples;
    int y = sampleIndex / sqrtSamples;

    float u_sample = (x + 0.5f) / static_cast<float>(sqrtSamples);
    float v_sample = (y + 0.5f) / static_cast<float>(sqrtSamples);

    // 计算面光源上的采样点
    glm::vec3 sampledPoint = light.position +
        (light.u * u_sample) +
        (light.v * v_sample);

    return sampledPoint;
}

float PowerHeuristic(const float& pdf_s, const float& n_s, const float& pdf_f, const float& n_f)
{
    if (isinf(pdf_s))
        return 1.0f;
    if (isinf(pdf_f))
        return 0.0f;
    else if (fabs(pdf_f - 0.0f) < 1e-6f && fabs(pdf_s - 0.0f) < 1e-6f)
        return 0.f;
    else
        return glm::pow(pdf_s * n_s, 2.0f) / (glm::pow(pdf_s * n_s, 2.0f) + glm::pow(pdf_f * n_f, 2.0f));
}

float calculateReflectionRate(
    const glm::vec3& rayDirection, // 入射光线方向
    const glm::vec3& normal,      // 法向量
    float nInterior,           // 内部介质折射率
    float nExterior            // 外部介质折射率
) {
    // 规范化输入方向和法向量
    glm::vec3 normalizedRayDir = glm::normalize(rayDirection);
    glm::vec3 normalizedNormal = glm::normalize(normal);

    // 确保法向量指向正确的方向
    float cosThetaI = std::clamp(glm::dot(normalizedRayDir, normalizedNormal), -1.0f, 1.0f);
    bool entering = cosThetaI < 0; // 判断是否从外部进入内部
    float n1 = entering ? nExterior : nInterior;
    float n2 = entering ? nInterior : nExterior;

    // 如果光线从内部射向外部，调整法向量方向
    if (!entering) {
        normalizedNormal = -normalizedNormal;
        cosThetaI = -cosThetaI;
    }
    cosThetaI = abs(cosThetaI);
    // 计算折射角的余弦值
    float eta = n1 / n2;
    float sinThetaT2 = eta * eta * (1.0 - cosThetaI * cosThetaI);

    // 如果发生全反射
    if (sinThetaT2 > 1.0) {
        return 1.0; // 完全反射
    }
    float cosThetaT = abs(std::sqrt(1.0 - sinThetaT2));

    // 计算菲涅尔反射率
    float Rs = (n1 * cosThetaI - n2 * cosThetaT) / (n1 * cosThetaI + n2 * cosThetaT);
    float Rp = (n2 * cosThetaI - n1 * cosThetaT) / (n2 * cosThetaI + n1 * cosThetaT);

    return (Rs * Rs + Rp * Rp) / 2.0; // 平均反射率
}


// 漫反射光照
glm::vec3 PathTracer::computeDiffuseLighting(Intersection& intersection, BVH& bvh, const Scene& scene, std::mt19937& gen, float &light_pdf) {
    glm::vec3 diffuseColor(0.0f, 0.0f, 0.0f);
	light_pdf = 0.0f;
    for (const auto& light : scene.lights) {
        // 判断是否为面光源（通过检查 u 和 v 向量）
        if (glm::length(light.u) > 0.0f && glm::length(light.v) > 0.0f) {
            //std::cout << "Light size: (" << light.u[0] << ", "<< light.u[1] <<", " << light.u[2] << "), (" << light.v[0] << ", " << light.v[1] << ", " << light.v[2] << ")" << std::endl;
            // 面光源：进行多次采样
            int numSamples = light.samples;
            glm::vec3 lightContribution(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < numSamples; ++i) {
                // 在面光源上采样一个点
                glm::vec3 sampledPoint = sampleAreaLight(light, i, numSamples);
                //std::cout << "smaplePoint: " << sampledPoint[0] << ", " << sampledPoint[1] << ", " << sampledPoint[2] << std::endl;

                // 计算从交点到采样点的方向和距离
                glm::vec3 lightDir = sampledPoint - intersection.point();
                float lightDistance = glm::length(lightDir);
                lightDir = glm::normalize(lightDir);

                // 计算光强衰减
                float attenuation = light.intensity / (lightDistance * lightDistance);

                // Compute the cosine of the angle between the light direction and the surface normal
                float cosTheta1 = glm::dot(intersection.normal(), lightDir);
                //std::cout << "cos1: " << cosTheta1 << std::endl;
                if (cosTheta1 <= 0.0f)
                    continue; // Light is below the surface
                glm::vec3 y = glm::vec3(0.0f, 1.0f, 0.0f);
                float cosTheta2 = glm::dot(lightDir, y);
                //std::cout << "cos2: " << cosTheta2 << std::endl;

                // 阴影射线：从交点到采样点
                float e = 0.0001f;
                Ray shadowRay = { intersection.point() + e * intersection.normal(), lightDir };
                Intersection shadowIntersection;
                float t = lightDistance - EPSILON;
                bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
                //if (inShadow)
                  //  std::cout << 1 << std::endl;
                //else
                  //  std::cout << 0 << std::endl;
                if (inShadow) {
                    continue; // 采样点被遮挡，跳过
                }

                // 计算漫反射贡献
                // float diff = glm::max(0.0f, glm::dot(intersection.normal(), lightDir));
                // lightContribution += intersection.material().diffuseReflect * diff * attenuation * light.color;

                //glm::vec3 brdf = intersection.material().diffuseReflect / cosTheta2;
                float area = glm::length(glm::cross(light.u, light.v));
                float pdf = 1.0f / area;
                //float cosineWeightedPDF = cosTheta1 * cosTheta2;
                //glm::vec3 weight = (brdf * cosineWeightedPDF * attenuation) / (pdf);
				glm::vec3 weight = intersection.material().diffuseReflect * cosTheta1 * attenuation / pdf;
                float pdfLight = pdf * lightDistance * lightDistance / fabs(cosTheta2);
				light_pdf = pdfLight / static_cast<float>(numSamples);
				float pdfBRDF = fabs(cosTheta1) / static_cast<float>(M_PI);
                float W = PowerHeuristic(pdfLight, static_cast<float>(numSamples), pdfBRDF, 1);
                //std::cout << "weight: " << weight[0] << ", " << weight[1] << ", " << weight[2] << std::endl;

                lightContribution +=  weight * light.color;
                //std::cout << "light: " << lightContribution[0] << ", " << lightContribution[1] << ", " << lightContribution[2] << std::endl;
            }

            // 将所有采样的贡献平均
            diffuseColor += lightContribution / static_cast<float>(numSamples);
        }
        else { // 点光源
            glm::vec3 lightDir = light.position - intersection.point();
            float lightDistance = glm::length(lightDir);
            float attenuation = light.intensity / (lightDistance * lightDistance);
            lightDir = glm::normalize(lightDir);

            // Compute the cosine of the angle between light direction and normal
            float cosTheta1 = glm::dot(intersection.normal(), lightDir);
            if (cosTheta1 <= 0.0f)
                continue; // Light is below the surface
            float cosThera2 = glm::dot(lightDir, glm::vec3(0.0f, 0.0f, -1.0f));

            // 阴影射线：从交点到光源位置
            float e = 0.0001f;
            Ray shadowRay = { intersection.point() + e * intersection.normal(), lightDir };
            Intersection shadowIntersection;
            float t = lightDistance - EPSILON;
            bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
            //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
            if (inShadow) {
                continue; // 被遮挡，跳过
            }

            //float diff = glm::max(0.0f, glm::dot(intersection.normal(), lightDir));
            //diffuseColor += intersection.material().diffuseReflect * diff * lightIntensity * light.color;

             // Compute BRDF (Lambertian)
            glm::vec3 brdf = intersection.material().diffuseReflect;

            // Compute PDF for point light sampling (delta distribution, handled via direct lighting)
            float pdf = glm::length(light.u) * glm::length(light.v);

            // Importance Sampling: direct illumination from point light
            glm::vec3 weight = (brdf * cosTheta1 * cosThera2) / pdf;

            diffuseColor += weight * light.color * attenuation;
        }
    }
    //std::cout << "Diffuse Color: " << diffuseColor[0] << " " << diffuseColor[1] << " " << diffuseColor[2] << std::endl;

    return diffuseColor;
}

// GGX distribution model
float GGX_D(const glm::vec3& h, const glm::vec3& n, float alpha) {
    float nh = glm::dot(n, h);
    return (alpha * alpha) / (M_PI * pow(nh * nh * (alpha * alpha - 1.0f) + 1.0f, 2));
}
float GGX_G(const glm::vec3& l, const glm::vec3& v, const glm::vec3& n, const glm::vec3& h) {
    float nl = glm::dot(n, l);
    float nv = glm::dot(n, v);
    float lh = glm::dot(l, h);
    float vh = glm::dot(v, h);

    float ggx1 = 2.0f * nl * lh / (lh + vh);
    float ggx2 = 2.0f * nv * vh / (vh + lh);

    return std::min(1.0f, std::min(ggx1, ggx2));
}
float GGX_F(const glm::vec3& v, const glm::vec3& h, float F0) {
    return F0 + (1.0f - F0) * pow(1.0f - glm::dot(v, h), 5);
}

// 计算导体材质的菲涅尔项  每个分量不等, eta F_0 需要是 glm::vec3
glm::vec3 PathTracer::computeFresnelConductor(float cosTheta, const glm::vec3& eta, const glm::vec3& k) {
    glm::vec3 eta2 = eta * eta;
    glm::vec3 k2 = k * k;
    glm::vec3 cosTheta2 = glm::vec3(cosTheta * cosTheta);

    glm::vec3 t0 = eta2 + k2;
    glm::vec3 t1 = t0 * cosTheta2;
    glm::vec3 t2 = 2.0f * eta * cosTheta;

    glm::vec3 rs = (t0 - t2 + cosTheta2) / (t0 + t2 + cosTheta2);
    glm::vec3 rp = (t1 - t2 + glm::vec3(1.0f)) / (t1 + t2 + glm::vec3(1.0f));

    return 0.5f * (rs + rp);  //when cosTheta = 0, rs = rp, return ((eta-k)^2+1)/(eta+k)^2+1; when k=0, return
}
// compute specular light
glm::vec3 PathTracer::computeSpecularLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray) {
    glm::vec3 specularColor(0.0f, 0.0f, 0.0f);  // initial specular color
    glm::vec3 viewDir = glm::normalize(ray.direction);  
    glm::vec3 normal = intersection.normal();  

    // Step 1: Handle two-sided material
    if (intersection.material().twoSided && glm::dot(ray.direction, normal) > 0) {   //back face
        normal = -normal;
    }

    // conductor specular light
    if (intersection.material().type == MaterialType::Conductor) {
        for (const auto& light : scene.lights) {
            // 判断是否为面光源
            if (glm::length(light.u) > 0.0f && glm::length(light.v) > 0.0f) {
                // 面光源：进行多次采样
                int numSamples = light.samples;
                glm::vec3 lightContribution(0.0f, 0.0f, 0.0f);

                for (int i = 0; i < numSamples; ++i) {
                    // 在面光源上采样一个点
                    glm::vec3 sampledPoint = sampleAreaLight(light, i, numSamples);

                    // 计算从交点到采样点的方向和距离
                    glm::vec3 lightDir = sampledPoint - intersection.point();
                    float lightDistance = glm::length(lightDir);
                    lightDir = glm::normalize(lightDir);

                    // 计算光强衰减
                    float attenuation = light.intensity / (lightDistance * lightDistance);

                    // 阴影射线：从交点到采样点
                    float e = 0.0001f;
                    Ray shadowRay = { intersection.point() + normal * e, lightDir };
                    Intersection shadowIntersection;
                    float t = lightDistance - EPSILON;
                    //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
                    bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                    if (inShadow) {
                        continue; // 采样点被遮挡，跳过
                    }

                    // Perfect reflection direction
                    glm::vec3 reflectDir = glm::normalize(glm::reflect(-lightDir, normal));
                    float spec = std::pow(std::max(0.0f, glm::dot(reflectDir, viewDir)), intersection.material().alpha);

                    lightContribution += intersection.material().specularReflect * spec * attenuation * light.color;
                }

                // 将所有采样的贡献平均
                specularColor += lightContribution / static_cast<float>(numSamples);
            }
            else { // 点光源
                glm::vec3 lightDir = light.position - intersection.point();  // ray direction
                float lightDistance = glm::length(lightDir);
                lightDir = glm::normalize(lightDir);

                // Check if the point is in shadow (if the light is blocked by other objects)
                float e = 0.0001f;
                Ray shadowRay = { intersection.point() + normal * e, lightDir };
                Intersection shadowIntersection;
                float t = lightDistance - EPSILON;
                bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
                if (inShadow) {
                    continue; // Point is in shadow, skip this light
                }

                // direction for perfect reflect
                glm::vec3 reflectDir = glm::normalize(glm::reflect(-lightDir, normal));
                float lightIntensity = light.intensity;
                float spec = std::pow(std::max(0.0f, glm::dot(reflectDir, viewDir)), intersection.material().alpha);

                specularColor += intersection.material().specularReflect * spec * (1.0f / (lightDistance * lightDistance)) * lightIntensity;
            }
        }
    }
    else {
        // material information
        float alpha = intersection.material().alpha;
        glm::vec3 specularReflect = intersection.material().specularReflect;

        // light contribute to specular term
        for (const auto& light : scene.lights) {
            // 判断是否为面光源
            if (glm::length(light.u) > 0.0f && glm::length(light.v) > 0.0f) {
                // 面光源：进行多次采样
                int numSamples = light.samples;
                glm::vec3 lightContribution(0.0f, 0.0f, 0.0f);

                for (int i = 0; i < numSamples; ++i) {
                    // 在面光源上采样一个点
                    glm::vec3 sampledPoint = sampleAreaLight(light, i, numSamples);

                    // 计算从交点到采样点的方向和距离
                    glm::vec3 lightDir = sampledPoint - intersection.point();
                    float lightDistance = glm::length(lightDir);
                    lightDir = glm::normalize(lightDir);

                    // 计算光强衰减
                    float attenuation = light.intensity / (lightDistance * lightDistance);

                    // 阴影射线：从交点到采样点
                    float e = 0.0001f;
                    Ray shadowRay = { intersection.point() + normal * e, lightDir };
                    Intersection shadowIntersection;
                    float t = lightDistance - EPSILON;
                    bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                    //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
                    if (inShadow) {
                        continue; // 采样点被遮挡，跳过
                    }

                    // 半程向量 h
                    glm::vec3 halfDir = glm::normalize(lightDir + viewDir);

                    // 材质信息
                    float intIOR = intersection.material().int_ior;
                    float extIOR = intersection.material().ext_ior;
                    glm::vec3 eta = intersection.material().eta;
                    float eta_f = 0.0f;
                    if (glm::length(eta) < EPSILON && intIOR > EPSILON) {
                        eta_f = intIOR / extIOR;
                    }
                    glm::vec3 k = intersection.material().k;

                    // 计算 GGX 分布 D, 几何遮蔽 G 和 菲涅尔项 F
                    float D = GGX_D(halfDir, normal, alpha);
                    float G = GGX_G(lightDir, viewDir, normal, halfDir);
                    glm::vec3 F = glm::vec3(0.0f);
                    float cosTheta = glm::dot(viewDir, halfDir);  // 计算视线方向与半程向量之间的夹角余弦
                    if (intersection.material().type == MaterialType::RoughConductor) {
                        // k != 0, 对导体材质（金属）的反射
                        F = computeFresnelConductor(cosTheta, eta, k);
                    }
                    else {
                        // k = 0, 对塑料，陶瓷的反射
                        float F0 = 0.0f;  // all the eta = 2/3
                        float F_f = GGX_F(viewDir, halfDir, F0);
                        F = glm::vec3(F_f, F_f, F_f);
                    }

                    // 计算有一定粗糙度的，GGX分布的镜面反射光照
                    auto spec = (D * G * F) / (4.0f * glm::dot(lightDir, normal) * glm::dot(viewDir, normal));

                    lightContribution += specularReflect * spec * attenuation;
                }

                // 将所有采样的贡献平均
                specularColor += lightContribution / static_cast<float>(numSamples);
            }
            else { // 点光源
                glm::vec3 lightDir = light.position - intersection.point();  // light direction 
                float lightDistance = glm::length(lightDir);
                lightDir = glm::normalize(lightDir);

                // Check if the point is in shadow (if the light is blocked by other objects)
                float e = 0.0001f;
                Ray shadowRay = { intersection.point() + normal * e, lightDir };
                Intersection shadowIntersection;
                float t = lightDistance - EPSILON;
                bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                if (inShadow) {
                    continue; // Point is in shadow, skip this light
                }

                // 半程向量 h
                glm::vec3 halfDir = glm::normalize(lightDir + viewDir);

                // 材质信息
                float intIOR = intersection.material().int_ior;
                float extIOR = intersection.material().ext_ior;
                glm::vec3 eta = intersection.material().eta;
                float eta_f = 0.0f;
                if (glm::length(eta) < EPSILON && intIOR > EPSILON) {
                    eta_f = intIOR / extIOR;
                }
                glm::vec3 k = intersection.material().k;

                // 计算 GGX 分布 D, 几何遮蔽 G 和 菲涅尔项 F
                float D = GGX_D(halfDir, normal, alpha);
                float G = GGX_G(lightDir, viewDir, normal, halfDir);
                glm::vec3 F = glm::vec3(0.0f);
                float cosTheta = glm::dot(viewDir, halfDir);  // 计算视线方向与半程向量之间的夹角余弦
                if (intersection.material().type == MaterialType::RoughConductor) {
                    // k != 0, 对导体材质（金属）的反射
                    F = computeFresnelConductor(cosTheta, eta, k);
                }
                else {
                    // k = 0, 对塑料，陶瓷的反射
                    float F0 = 0.0f;  // all the eta = 2/3
                    float F_f = GGX_F(viewDir, halfDir, F0);
                    F = glm::vec3(F_f, F_f, F_f);
                }

                // 计算有一定粗糙度的，GGX分布的镜面反射光照
                auto spec = (D * G * F) / (4.0f * glm::dot(lightDir, normal) * glm::dot(viewDir, normal));

                specularColor += specularReflect * spec * (1.0f / (lightDistance * lightDistance));
            }
        }
    }

    //std::cout << "SpecularColor: " << specularColor[0] << " " << specularColor[1] << " " << specularColor[2] << std::endl;
    return specularColor;
}

// refraction direction
glm::vec3 PathTracer::refractDirection(const glm::vec3& incident, const glm::vec3& normal, float ext_ior, float int_ior) {
    float eta = ext_ior / int_ior; 

    // cos for interior
    float cosi = glm::clamp(glm::dot(incident, normal), -1.0f, 1.0f);

    // whether exists total internal reflection
    float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
    if (k < 0.0f) {
        return glm::vec3(0.0f); // Total internal reflection, no refraction
    }
    else {
        return eta * incident - (eta * cosi + sqrt(k)) * normal;
    }
}
// refraction light
glm::vec3 PathTracer::computeRefractionLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray) {
    glm::vec3 refractionColor(0.0f, 0.0f, 0.0f);  
    glm::vec3 viewDir = glm::normalize(ray.direction); 
    glm::vec3 normal = intersection.normal(); 

    // Step 1: Handle two-sided material
    if (intersection.material().twoSided && glm::dot(ray.direction, normal) > 0) {   //back face
        normal = -normal;
    }
    for (const auto& light : scene.lights) {
        // 判断是否为面光源
        if (glm::length(light.u) > 0.0f && glm::length(light.v) > 0.0f) {
            // 面光源：进行多次采样
            int numSamples = light.samples;
            glm::vec3 lightContribution(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < numSamples; ++i) {
                // 在面光源上采样一个点
                glm::vec3 sampledPoint = sampleAreaLight(light, i, numSamples);

                // 计算从交点到采样点的方向和距离
                glm::vec3 lightDir = sampledPoint - intersection.point();
                float lightDistance = glm::length(lightDir);
                lightDir = glm::normalize(lightDir);

                // 计算光强衰减
                float attenuation = light.intensity / (lightDistance * lightDistance);

                // 阴影射线：从交点到采样点
                float e = 0.0001f;
                Ray shadowRay = { intersection.point() + normal * e, lightDir };
                Intersection shadowIntersection;
                float t = lightDistance - EPSILON;
                bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                if (inShadow) {
                    continue; // 采样点被遮挡，跳过
                }

                // 计算折射贡献
                float intIOR = intersection.material().int_ior;
                float extIOR = intersection.material().ext_ior;
                float eta_f = intIOR / extIOR;
                float F0 = 0.01f;

                glm::vec3 halfDir = glm::normalize(lightDir + viewDir);
                float fresnelTerm = GGX_F(viewDir, halfDir, F0);

                // 将折射贡献累积
                lightContribution += attenuation * light.color;
            }

            // 将所有采样的贡献平均
            refractionColor += lightContribution / static_cast<float>(numSamples);
        }
        else { // 点光源
            glm::vec3 lightDir = light.position - intersection.point();
            float lightDistance = glm::length(lightDir);
            float lightIntensity = light.intensity / (lightDistance * lightDistance);
            lightDir = glm::normalize(lightDir);

            // 阴影射线：从交点到光源位置
            float e = 0.0001f;
            Ray shadowRay = { intersection.point() + normal * e, lightDir };
            Intersection shadowIntersection;
            float t = lightDistance - EPSILON;
            bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
            if (inShadow) {
                continue; // Point is in shadow, skip this light
            }

            float intIOR = intersection.material().int_ior;
            float extIOR = intersection.material().ext_ior;
            float eta_f = intIOR / extIOR;
            float F0 = 0.04f;

            glm::vec3 halfDir = glm::normalize(lightDir + viewDir);
            float fresnelTerm = GGX_F(viewDir, halfDir, F0);

            // 将折射贡献累积
            refractionColor += (1.0f - fresnelTerm) * lightIntensity * light.color;
        }
    }

    // std::cout << "RefractionColor: " << refractionColor[0] << " " << refractionColor[1] << " " << refractionColor[2] << std::endl;
    return refractionColor;
}


// importance sampling
// cosine-weighted hemisphere sampling for diffuse reflection
glm::vec3 PathTracer::generateRandomDirection(glm::vec3 normal, std::mt19937& gen) {
    std::uniform_real_distribution<float> distribution(0.0, 1.0);

    float r1 = 2 * M_PI * distribution(gen);  // 随机方位角
    float r2 = distribution(gen);             // 随机半径平方
    float r2s = std::sqrt(r2);                // 随机半径

    // 计算基于法向量的坐标系
    glm::vec3 w = glm::normalize(normal);
    glm::vec3 u = (fabs(w.x) > 0.1 ? glm::vec3(0, 1, 0) : glm::normalize(glm::cross(glm::vec3(1, 0, 0), w)));
    glm::vec3 v = glm::cross(w, u);

    // 计算随机方向向量
    //glm::vec3 d = (u * std::cos(r1) * r2s + v * std::sin(r1) * r2s + w * glm::sqrt(1.0f - r2));
    glm::vec3 d = (u * std::cos(r1) * r2s + v * std::sin(r1) * r2s + w * glm::sqrt(1.0f - r2));

    return glm::normalize(d);
}


// 追踪路径
glm::vec3 PathTracer::tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount, std::mt19937& gen) {
    std::uniform_real_distribution<> dis(0.0, 1.0);
    glm::vec3 beta = glm::vec3(1.0f);
    if (bounceCount >= MAX_BOUNCES)
        return glm::vec3(0.0f);

    glm::vec3 result_color = glm::vec3(0.0f);
    float light_pdf = 0.0f;
    Intersection intersection_scene;
    float t = 1e6f;  // init t set to a large value
    bool intersect_scene = bvh.intersect(ray, intersection_scene, 0.0f, t, scene.spheres);
 
    if (intersect_scene) {
        // std::cout << std::endl << std::endl << "Intersected!" << std::endl;
        const Material material_intersect = intersection_scene.material();
        glm::vec3 position_new = intersection_scene.point();
        glm::vec3 normal = intersection_scene.normal();

        // 处理双面材质
        if (material_intersect.twoSided && glm::dot(ray.direction, normal) > 0) {
            normal = -normal;
        }

        // 如果交点是一个发光体，累积其发光颜色
        if (glm::length(material_intersect.emission) > 1e-4) {
            // std::cout << "Intersect with area light!." << std::endl;
            result_color += material_intersect.emission;
			//std::cout << "Emission: " << material_intersect.emission[0] << " " << material_intersect.emission[1] << " " << material_intersect.emission[2] << std::endl;
            return result_color;
        }

        // 纯漫反射
        if (material_intersect.type == MaterialType::Diffuse) {
            result_color += computeDiffuseLighting(intersection_scene, bvh, scene, gen, light_pdf);
        }
        // 纯镜面反射
        else if (material_intersect.type == MaterialType::Conductor || material_intersect.type == MaterialType::RoughConductor) {
            result_color += computeSpecularLighting(intersection_scene, bvh, scene, ray);
        }
        // 透射反射
        else if (material_intersect.type == MaterialType::RoughDielectric) {
			/*float R = calculateReflectionRate(ray.direction, normal, material_intersect.int_ior, material_intersect.ext_ior);
            if (1 - R < EPSILON) {
                result_color += computeSpecularLighting(intersection_scene, bvh, scene, ray);
            }
            else {
				result_color += R * computeSpecularLighting(intersection_scene, bvh, scene, ray) + (1 - R) * computeRefractionLighting(intersection_scene, bvh, scene, ray);
            }*/
			result_color += computeRefractionLighting(intersection_scene, bvh, scene, ray);
        }
		// 漫反射镜面反射
        else {
            //std::cout << "Interior: " << material_intersect.int_ior << " Outerior: " << material_intersect.ext_ior << std::endl;
            float R0 = calculateReflectionRate(ray.direction, normal, material_intersect.int_ior, material_intersect.ext_ior);
			//std::cout << "R0: " << R0 << std::endl;
			float cosTheta = abs(glm::dot(ray.direction, normal));
			float R = R0 + (1 - R0) * std::pow(1 - cosTheta, 5.0f); // Schlick's approximation
			//std::cout << "R: " << R << std::endl;
			result_color += R * computeSpecularLighting(intersection_scene, bvh, scene, ray) + (1 - R) * computeDiffuseLighting(intersection_scene, bvh, scene, gen, light_pdf);
        }

        // 镜面反射光照
        /*if (glm::length(material_intersect.specularReflect) > EPSILON) {
            result_color += computeSpecularLighting(intersection_scene, bvh, scene, ray);
        }

        // 折射光照
        if (material_intersect.int_ior > EPSILON && material_intersect.ext_ior > EPSILON) {
            result_color += computeRefractionLighting(intersection_scene, bvh, scene, ray);
        }*/

        // Russian Roulette for Path Termination
        if (bounceCount >= 16) {
            float p = P;
            std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
            float rand = distribution(gen);
            if (rand > p) {
                return result_color;
            }
            beta /= p;
        }

        int bounceCount_new = bounceCount + 1;
        float e = 0.0001f;

        float pdf;
        glm::vec3 brdf;

        // 递归反射
        if (material_intersect.type == MaterialType::Conductor || material_intersect.type == MaterialType::RoughConductor) {
            //std::cout << "Intersect with specular material." << std::endl;
            glm::vec3 reflect_dir = glm::reflect(ray.direction, normal);
            Ray reflect_ray = { position_new + e * normal, reflect_dir };
			//brdf = material_intersect.specularReflect / glm::dot(reflect_dir, normal);
            //pdf = 1.0f;
            glm::vec3 beta_spec = beta * material_intersect.specularReflect;
            result_color += tracePath(reflect_ray, scene, bvh, bounceCount_new, gen) * beta_spec;
        } 
        else if (material_intersect.type == MaterialType::Diffuse) {
            //std::cout << "Intersect with diffuse material." << std::endl;
            pdf = 1 / static_cast<float>(2 * M_PI);
            glm::vec3 diffuse_dir = generateRandomDirection(normal, gen);
            Ray diffuse_ray = { position_new + e * normal, diffuse_dir };
            //brdf = material_intersect.diffuseReflect / static_cast<float>(M_PI);
            float cosTheta = glm::dot(normal, diffuse_dir);
			float brdf_pdf = abs(cosTheta) / static_cast<float>(M_PI);
            float W;
            if (glm::length(light_pdf) < EPSILON) {
				W = 1.0f;
			}
            else {
                W = PowerHeuristic(brdf_pdf, 1, light_pdf, 32);
            }
            glm::vec3 beta_diff = beta * material_intersect.diffuseReflect;
            result_color +=  tracePath(diffuse_ray, scene, bvh, bounceCount_new, gen) * beta_diff;
        }
        else if (material_intersect.type == MaterialType::RoughDielectric) {
            /*float R = calculateReflectionRate(ray.direction, normal, material_intersect.int_ior, material_intersect.ext_ior);
            float x = dis(gen);
            if (x <= R) {
                glm::vec3 reflect_dir = glm::reflect(ray.direction, normal);
                Ray reflect_ray = { position_new + e * normal, reflect_dir };
                glm::vec3 beta_spec = beta * material_intersect.specularReflect;
                result_color += tracePath(reflect_ray, scene, bvh, bounceCount_new, gen) * beta_spec;
            }
            else {
                glm::vec3 refract_dir = refractDirection(ray.direction, normal, material_intersect.ext_ior, material_intersect.int_ior);
                if (glm::length(refract_dir) > 0.0f) {
                    Ray refract_ray = { position_new - e * normal, refract_dir };
                    float cosTheta = glm::dot(normal, refract_dir);
                    glm::vec3 beta_ref = beta * cosTheta / (2 * static_cast<float> (M_PI));
                    result_color += tracePath(refract_ray, scene, bvh, bounceCount_new, gen) * beta_ref;
                }
            }*/
            glm::vec3 refract_dir = refractDirection(ray.direction, normal, material_intersect.ext_ior, material_intersect.int_ior);
            if (glm::length(refract_dir) > EPSILON) {
                Ray refract_ray = { position_new - e * normal, refract_dir };
                float cosTheta = glm::dot(normal, refract_dir);
                glm::vec3 beta_ref = beta * cosTheta / (2 * static_cast<float> (M_PI));
                result_color += tracePath(refract_ray, scene, bvh, bounceCount_new, gen) * beta;
            }
        }
        else {
            float R0 = calculateReflectionRate(ray.direction, normal, material_intersect.int_ior, material_intersect.ext_ior);
            float cosTheta = abs(glm::dot(ray.direction, normal));
            float R = R0 + (1 - R0) * std::pow(1 - cosTheta, 5.0f); // Schlick's approximation
            float x = dis(gen);
            if (x <= R) {
                glm::vec3 reflect_dir = glm::reflect(ray.direction, normal);
                Ray reflect_ray = { position_new + e * normal, reflect_dir };
                glm::vec3 beta_spec = beta * R;
				//std::cout << "R: " << R << std::endl;
                result_color += tracePath(reflect_ray, scene, bvh, bounceCount_new, gen) * beta_spec;
            }
            else {
                pdf = 1 / static_cast<float>(2 * M_PI);
                glm::vec3 diffuse_dir = generateRandomDirection(normal, gen);
                Ray diffuse_ray = { position_new + e * normal, diffuse_dir };
                float cosTheta = glm::dot(normal, diffuse_dir);
                float brdf_pdf = abs(cosTheta) / static_cast<float>(M_PI);
                float W;
                if (glm::length(light_pdf) < EPSILON) {
                    W = 1.0f;
                }
                else {
                    W = PowerHeuristic(brdf_pdf, 1, light_pdf, 32);
                }
                glm::vec3 beta_diff = beta * material_intersect.diffuseReflect;
                result_color += tracePath(diffuse_ray, scene, bvh, bounceCount_new, gen) * beta_diff;
            }
        }
    }
    else if (useHDR) {
        // 如果没有相交并启用了 HDR，使用 HDR 纹理
        //std::cout << "No intersection, use HDR." << std::endl;
        return sampleEquirectangularMap(environmentMap, ray.direction);
    }
    else {
        // 没有相交也没有 HDR，返回背景色
        //std::cout << "No intersection, don't use HDR." << std::endl;
        return glm::vec3(0.0f);
    }

    return result_color;
}

// generate a sample
// for anti-aliasing
glm::vec3 generateSample(const Camera& camera, int x, int y, int width, int height, std::mt19937& gen) {
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
    float scale = tan(glm::radians(camera.fov * 0.5f));

    float u = (2.0f * ((x + dis(gen)) / static_cast<float>(width)) - 1.0f) * aspect_ratio * scale;
    float v = (1.0f - 2.0f * ((y + dis(gen)) / static_cast<float>(height))) * scale;

    glm::vec3 forward = glm::vec3(0.0, 0.0, 1.0);
    glm::vec3 right = glm::vec3(1.0, 0.0, 0.0);
    glm::vec3 up = glm::vec3(0.0, 1.0, 0.0);

    glm::vec3 sample_direction = glm::normalize(glm::transpose(camera.rotationMatrix) * (forward + u * right + v * up));
    //std::cout << "Sample direction: " << sample_direction[0] << " " << sample_direction[1] << " " << sample_direction[2] << std::endl;
    return sample_direction;
}


void PathTracer::renderWorker(const Scene& scene, const Camera& camera, BVH& bvh, int width, int height, int samplesPerPixel,
    std::vector<glm::vec3>& framebuffer, std::mt19937& gen) {
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);

    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this] {return !task_queue.empty() || done; });

            if (task_queue.empty()) {
                if (done) return;
                else continue;
            }

            task = task_queue.front();
            task_queue.pop();
        }

        for (int y = task.yStart; y < task.yEnd; y++) {
            for (int x = task.xStart; x < task.xEnd; x++) {
                if (x < 0 || x >= width || y < 0 || y >= height) {
                    std::lock_guard<std::mutex> lock(cout_mutex);
                    std::cerr << "Pixel index out-of-bound: (" << x << ", " << y << ")\n";
                    continue;
                }

                glm::vec3 pixel_radiance(0.0f, 0.0f, 0.0f);

                for (int i = 0; i < samplesPerPixel; i++) {
                    //PROFILE("sample one pixel")
                    glm::vec3 sample_direction = generateSample(camera, x, y, width, height, gen);
                    Ray ray = { camera.position, sample_direction };
                    glm::vec3 beta = glm::vec3(1.0f);
                    glm::vec3 color_mid = tracePath(ray, scene, bvh, 0, gen);
                    pixel_radiance += color_mid * (1.0f / static_cast<float>(samplesPerPixel));
                }
                //if (y >= 100) {
                  //  {
                    //    std::lock_guard<std::mutex> lock(cout_mutex);
                      //  std::cout << "Finish render pixel: (" << x << ", " << y << ")\n";
                    //}
                //}

                framebuffer[y * width + x] += pixel_radiance;
            }
        }
    }
}

// 主路径追踪函数
void PathTracer::render(const Scene& scene, const Camera& camera, BVH& bvh,
    int width, int height, int samplesPerPixel,
    int numThreads) {
    PROFILE("Render " + std::to_string(width) + "x" + std::to_string(height) + " " +  std::to_string(samplesPerPixel) + "spp")
    // 确保 framebuffer 大小正确
    std::vector<glm::vec3> framebuffer(width * height, glm::vec3(0.0f));
    std::vector<std::thread> threadPool;

    if (useHDR) {
        std::cout << "Using HDR environment lighting..." << std::endl;
    }

    const int xtileSize = width / 10;
    const int ytileSize = height / 10;
    //std::cout << "xtileSize: " << xtileSize << "ytileSize: " << ytileSize << std::endl;

    std::vector<unsigned int> seeds(numThreads);
    std::mt19937 rd_gen(std::random_device{}());
    for (auto& seed : seeds) {
        seed = rd_gen();
    }

    // 计算每个线程处理的像素列范围
    // int colsPerThread = height / numThreads;
    // int remainingCols = height % numThreads;

    for (int t = 0; t < numThreads; ++t) {
        std::mt19937 thread_gen(seeds[t]);
        threadPool.emplace_back(&PathTracer::renderWorker, this, std::ref(scene), std::ref(camera), std::ref(bvh),
            width, height, samplesPerPixel, std::ref(framebuffer), std::ref(thread_gen));
    }

    enqueueTasks(xtileSize, ytileSize, width, height);

    // 等待所有线程完成
    for (auto& thread : threadPool)
        thread.join();

    // 输出图像到文件（如.ppm格式）
    std::ofstream outFile("output.ppm");
    if (!outFile.is_open()) {
        std::cerr << "Failed to open output file.\n";
        return;
    }
    outFile << "P3\n" << width << " " << height << "\n255\n";
    for (auto& color : framebuffer) {
        // 进行伽马校正或其他颜色空间转换（可选）
        color = glm::pow(color, glm::vec3(1.0f / 2.2f));
        glm::vec3 clamped = glm::clamp(color, 0.0f, 1.0f);
        outFile << static_cast<int>(clamped.r * 255.0f) << " "
            << static_cast<int>(clamped.g * 255.0f) << " "
            << static_cast<int>(clamped.b * 255.0f) << "\n";
    }
    outFile.close();

    std::cout << "Rendering complete. Image saved to output.ppm." << std::endl;
}