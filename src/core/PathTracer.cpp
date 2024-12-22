#include "PathTracer.h"
#define EPSILON 1e-6
#define M_PI 3.1415926535

const int MAX_BOUNCES = 8;
const float P = 0.66666;

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
glm::vec3 bilinearInterpolation(const glm::vec2& uv, std::shared_ptr<Texture> texture) {
    int width = texture->width, height = texture->height;
    // uv 是归一化的纹理坐标，范围在 [0, 1] 之间
    float x = uv.x * (width - 1);  // 将 uv.x 映射到纹理的水平坐标
    float y = uv.y * (height - 1); // 将 uv.y 映射到纹理的垂直坐标

    int x0 = static_cast<int>(x);   // 左侧网格点
    int y0 = static_cast<int>(y);   // 底部网格点
    int x1 = std::min(x0 + 1, width - 1); // 右侧网格点（防止越界）
    int y1 = std::min(y0 + 1, height - 1); // 顶部网格点（防止越界）

    glm::vec3 color(0.0f);
    for (int i = 0; i < 3; ++i) {
        // 获取四个邻近点的纹理颜色
        int id1 = (y0 * width + x0) * 3 + i;
        int id2 = (y0 * width + x1) * 3 + i;
        int id3 = (y1 * width + x0) * 3 + i;
        int id4 = (y1 * width + x1) * 3 + i;

        /*std::cout << "id1: " << id1 << " value: " << (int)texture->data[id1] << std::endl;
        std::cout << "id2: " << id2 << " value: " << (int)texture->data[id2] << std::endl;
        std::cout << "id3: " << id3 << " value: " << (int)texture->data[id3] << std::endl;
        std::cout << "id4: " << id4 << " value: " << (int)texture->data[id4] << std::endl;*/


        // 水平插值 (x方向)
        float R1 = texture->data[id1] + (x - x0) * (texture->data[id2] - texture->data[id1]); // f(x, y0)
        float R2 = texture->data[id3] + (x - x0) * (texture->data[id4] - texture->data[id3]); // f(x, y1)

        //std::cout << "R1: " << R1 << " R2: " << R2 << std::endl;

        // 垂直插值 (y方向)
        float value = R1 + (y - y0) * (R2 - R1);

        color[i] = value / 255.0f; // 转换为 0-1 范围
    }

    return color;
}


// 漫反射光照
glm::vec3 PathTracer::computeDiffuseLighting(Intersection& intersection, BVH& bvh, const Scene& scene, std::mt19937& gen) {
    glm::vec3 diffuseColor(0.0f, 0.0f, 0.0f);

    for (const auto& light : scene.lights) {
        // 判断是否为面光源（通过检查 u 和 v 向量）
        if (glm::length(light.u) > 0.0f && glm::length(light.v) > 0.0f) {
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
                if (cosTheta1 <= 0.0f)
                    continue; 
                glm::vec3 y = glm::vec3(0.0f, 1.0f, 0.0f);
                float cosTheta2 = glm::dot(lightDir, y);

                 //阴影射线：从交点到采样点
                float e = 0.0001f;
                Ray shadowRay = { intersection.point() + e * intersection.normal(), lightDir };
                Intersection shadowIntersection;
                float t = lightDistance - EPSILON;
                bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                if (inShadow) {
                    continue; // 采样点被遮挡，跳过
                }

                if (intersection.material().IsTexture) {
                    glm::vec2 uv = intersection.uv();
                    glm::vec3 textureColor = bilinearInterpolation(uv, intersection.material().texture);
                    intersection.material().diffuseReflect = textureColor;
                    /*if (intersection.material().diffuseReflect == glm::vec3(0.0f)) {
                        std::cout << "NO texture COLOR!" << std::endl;
                    }*/
                }

                // compute BRDF (Lambertian)
                //glm::vec3 brdf = intersection.material().diffuseReflect / cosTheta2;
                 // compute PDF for area light sampling (uniform over area)
                float area = glm::length(glm::cross(light.u, light.v));
                float pdf = 1.0f / area;
                //std::cout << "pdf: " << pdf << std::endl;

                // Importance Sampling: cosine-weighted distribution aligns with BRDF
                //float cosineWeightedPDF = cosTheta1 * cosTheta2;
                //glm::vec3 weight = (brdf * cosTheta1) / (pdf);
                glm::vec3 weight = intersection.material().diffuseReflect * cosTheta1 * attenuation / pdf;
                lightContribution += weight * light.color;
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

            //texture mapping
            if (intersection.material().IsTexture) {
                glm::vec2 uv = intersection.uv();
                glm::vec3 textureColor = bilinearInterpolation(uv, intersection.material().texture);
                intersection.material().diffuseReflect = textureColor;
            }

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
		intersection.normal_ = -normal;
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
                //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
                bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
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
                    //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
                    bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
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
                //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
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
                //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
                bool inShadow = bvh.intersect(shadowRay, shadowIntersection, 0.0f, t, scene.spheres);
                if (inShadow) {
                    continue; // 采样点被遮挡，跳过
                }

                // 计算折射贡献
                float intIOR = intersection.material().int_ior;
                float extIOR = intersection.material().ext_ior;
                float eta_f = intIOR / extIOR;
                float F0 = 0.04f;

                glm::vec3 halfDir = glm::normalize(lightDir + viewDir);
                float fresnelTerm = GGX_F(viewDir, halfDir, F0);

                // 将折射贡献累积
                lightContribution += (1.0f - fresnelTerm) * attenuation * light.color;
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
            //bool inShadow = bvh.intersect(shadowRay, shadowIntersection, t);
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
    glm::vec3 d = (u * std::cos(r1) * r2s + v * std::sin(r1) * r2s + w * glm::sqrt(1.0f - r2));

    return glm::normalize(d);

}


// 追踪路径
glm::vec3 PathTracer::tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount, std::mt19937& gen) {
    glm::vec3 beta = glm::vec3(1.0f);
    if (bounceCount >= MAX_BOUNCES)
        return glm::vec3(0.0f);

    glm::vec3 result_color = glm::vec3(0.0f);
    Intersection intersection_scene;
    float t = 1e6f;  // init t set to a large value
    bool intersect_scene = bvh.intersect(ray, intersection_scene, 0.0f, t, scene.spheres);
    if (!intersect_scene) {
        // std::cout << "No intersection." << std::endl;
        return result_color;
    }

 //   // 处理双面材质
 //   if (intersection_scene.material().twoSided && glm::dot(ray.direction, intersection_scene.normal()) > 0) {
 //       intersection_scene.normal_ = -intersection_scene.normal();
 //   }

	//// for debugging the normal of the intersection
 //   if (intersect_scene) {
 //       return intersection_scene.normal_;
 //   }

    if (intersect_scene) {
        // std::cout << std::endl << std::endl << "Intersected!" << std::endl;
        Material material_intersect = intersection_scene.material();
        glm::vec3 position_new = intersection_scene.point();
        glm::vec3 normal = intersection_scene.normal();
        glm::vec2 uv = intersection_scene.uv();
        bool IsTexture = intersection_scene.material().IsTexture;

   //     // 处理双面材质
        if (material_intersect.twoSided && glm::dot(ray.direction, normal) > 0) {
            intersection_scene.normal_ = -intersection_scene.normal();
            normal = intersection_scene.normal();
        }

        // 如果交点是一个发光体，累积其发光颜色
        if (glm::length(material_intersect.emission) > 1e-4) {
            // std::cout << "Intersect with area light!." << std::endl;
            result_color += material_intersect.emission;
			//std::cout << "Emission: " << material_intersect.emission[0] << " " << material_intersect.emission[1] << " " << material_intersect.emission[2] << std::endl;
            return result_color;
        }

   //     // 漫反射光照
        if (glm::length(material_intersect.diffuseReflect) > EPSILON || IsTexture) {
            result_color += computeDiffuseLighting(intersection_scene, bvh, scene, gen);
            material_intersect.diffuseReflect = intersection_scene.material().diffuseReflect;
        }

   //     // 镜面反射光照
        if (glm::length(material_intersect.specularReflect) > EPSILON) {
            result_color += computeSpecularLighting(intersection_scene, bvh, scene, ray);
        }

        // 折射光照
        /*if (material_intersect.int_ior > EPSILON && material_intersect.ext_ior > EPSILON) {
            result_color += computeRefractionLighting(intersection_scene, bvh, scene, ray);
        }*/

        // Russian Roulette for Path Termination
        if (bounceCount >= 5) {
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
        if (glm::length(material_intersect.specularReflect) > EPSILON) {
            //std::cout << "Intersect with specular material." << std::endl;
            glm::vec3 reflect_dir = glm::reflect(ray.direction, normal);
            Ray reflect_ray = { position_new + e * normal, reflect_dir };
            //brdf = material_intersect.specularReflect / glm::dot(reflect_dir, normal);
            //pdf = 1.0f;
            beta *= material_intersect.specularReflect;
            float cosTheta = glm::dot(normal, reflect_dir);
            result_color += tracePath(reflect_ray, scene, bvh, bounceCount_new, gen) * beta;
        }

        // 递归漫反射
        if (glm::length(material_intersect.diffuseReflect) > EPSILON || IsTexture) {
            //std::cout << "Intersect with diffuse material." << std::endl;
            pdf = 1 / static_cast<float> (2 * M_PI);
            glm::vec3 diffuse_dir = generateRandomDirection(normal, gen);
            Ray diffuse_ray = { position_new + e * normal, diffuse_dir };
            //brdf = material_intersect.diffuseReflect / static_cast<float>(M_PI);
            //beta *= brdf * abs(glm::dot(diffuse_dir, normal)) / pdf;
            beta *= material_intersect.diffuseReflect * abs(glm::dot(diffuse_dir, normal)) * 2.0f;
            float cosTheta = glm::dot(normal, diffuse_dir);
            result_color += tracePath(diffuse_ray, scene, bvh, bounceCount_new, gen) * beta;
        }

        // 递归折射
        //if (material_intersect.int_ior > EPSILON && material_intersect.ext_ior > EPSILON) {
        //    //std::cout << "Intersect with refraction materail." << std::endl;
        //    glm::vec3 refract_dir = refractDirection(ray.direction, normal, material_intersect.ext_ior, material_intersect.int_ior);
        //    if (glm::length(refract_dir) > 0.0f) {
        //        Ray refract_ray = { position_new - e * normal, refract_dir };
        //        float cosTheta = glm::dot(normal, refract_dir);
        //        result_color += tracePath(refract_ray, scene, bvh, bounceCount_new, gen) * cosTheta / (2 * static_cast<float> (M_PI));
        //    }
        //}
    }
     //else {
         // 背景颜色，可选
         // result_color += bg_color;
     //}

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

// 渲染图像的一部分，无需互斥锁，因为每个线程处理独立的像素区域
/*void PathTracer::renderSection(const Scene& scene, const Camera& camera, BVH& bvh,
    int width, int height, int samplesPerPixel,
    int yStart, int yEnd, std::vector<glm::vec3>& framebuffer,
    std::mt19937& gen) {

    std::uniform_real_distribution<float> dis(0.0f, 1.0f);

    for (int x = 0; x < width; ++x) {
        for (int y = yStart; y < yEnd; y++) {
            if (x < 0 || x >= width || y < 0 || y >= height) {
                std::cerr << "Pixel index out-of-bounds: (" << x << ", " << y << ")\n";
                continue;
            }
            glm::vec3 pixel_radiance(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < samplesPerPixel; ++i) {
                glm::vec3 sample_direction = generateSample(camera, x, y, width, height, gen);
                Ray ray = { camera.position, sample_direction };
                glm::vec3 color_mid = tracePath(ray, scene, bvh, 0, gen);
                pixel_radiance += color_mid * (1.0f / static_cast<float>(samplesPerPixel));
            }

            // 建议移除调试输出以提升性能和避免竞争
            
            std::cout << "Finish rendering pixel (" << x << ", " << y << ")." << std::endl;
            // std::cout << "Color: " << pixel_radiance[0] << " " << pixel_radiance[1] << " " << pixel_radiance[2] << std::endl;

            // 直接写入帧缓冲，无需互斥锁
            framebuffer[y * width + x] += pixel_radiance;
        }
    }
}*/

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