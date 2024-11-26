#include "PathTracer.h"
#define EPSILON 1e-6
#define M_PI 3.1415926535

const int MAX_BOUNCES = 16;
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> dis(0.0f, 1.0f);

//point light
bool intersectLight(const Ray& ray, const std::vector<Light> lights, Light& result_light, Intersection& intersection) {
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
}

// 漫反射光照
glm::vec3 PathTracer::computeDiffuseLighting(Intersection& intersection, BVH& bvh, const Scene& scene) {
    glm::vec3 diffuseColor(0.0f, 0.0f, 0.0f);

    for (const auto& light : scene.lights) {
        glm::vec3 lightDir = light.position - intersection.point(); 
        float lightDistance = glm::length(lightDir);
        float lightIntensity = light.intensity / (lightDistance * lightDistance);
        lightDir = glm::normalize(lightDir);

        // Check if the point is in shadow (if the light is blocked by other objects)
        Ray shadowRay = { intersection.point(), lightDir };
        Intersection shadowIntersection = Intersection();
        float t = 1e6;
        bool hit = bvh.intersect(shadowRay, shadowIntersection, t);
        if (hit) {
            continue; // Point is in shadow, skip this light
        }

        float diff = std::max(0.0f, glm::dot(intersection.normal(), lightDir));
        diffuseColor += intersection.material()->diffuseReflect * diff * lightIntensity;  //��intensity

    }

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

    float ggx1 = 2.0f * nl * lh / lh;
    float ggx2 = 2.0f * nv * vh / vh;

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
    if (intersection.material()->twoSided && glm::dot(ray.direction, normal) > 0) {   //back face
        normal = -normal;
    }

    // conductor specular light
    if (intersection.material()->type == MaterialType::Conductor) {
        for (const auto& light : scene.lights) {
            glm::vec3 lightDir = light.position - intersection.point();  // ray direction
            float lightDistance = glm::length(lightDir);
            lightDir = glm::normalize(lightDir);

            // Check if the point is in shadow (if the light is blocked by other objects)
            Ray shadowRay = { intersection.point(), lightDir };
            Intersection shadowIntersection = Intersection();
            float t = 1e6;
            bool hit = bvh.intersect(shadowRay, shadowIntersection, t);
            if (hit) {
                continue; // Point is in shadow, skip this light
            }

            // direction for perfect reflect
            glm::vec3 reflectDir = glm::normalize(glm::reflect(-lightDir, normal));
            float lightIntensity = light.intensity;
            float spec = std::pow(std::max(0.0f, glm::dot(reflectDir, lightDir)), intersection.material()->alpha);

            // 菲涅尔项 nonlinear
            //float cosTheta = glm::dot(viewDir, normal);
            //float fresnel = pow(1.0f - cosTheta, 5);  //(1-cosTheta)^5

            specularColor += intersection.material()->specularReflect * spec * (1 / (lightDistance * lightDistance)) * lightIntensity;
        }
    }
    else {
        // material information
        float alpha = intersection.material()->alpha;
        glm::vec3 specularReflect = intersection.material()->specularReflect;

        // light contribute to specular term
        for (const auto& light : scene.lights) {
            glm::vec3 lightDir = light.position - intersection.point();  // light direction 
            float lightDistance = glm::length(lightDir);
            lightDir = glm::normalize(lightDir);

            // Check if the point is in shadow (if the light is blocked by other objects)
            Ray shadowRay = { intersection.point(), lightDir };
            Intersection shadowIntersection = Intersection();
            float t = 1e6;
            bool hit = bvh.intersect(shadowRay, shadowIntersection, t);
            if (hit) {
                continue; // Point is in shadow, skip this light
            }

            // 半程向量 h
            glm::vec3 halfDir = glm::normalize(lightDir + viewDir);

            // 有粗糙度的镜面反射 or 非线性材质
            float intIOR = intersection.material()->int_ior;
            float extIOR = intersection.material()->ext_ior;
            glm::vec3 eta = intersection.material()->eta;
            float eta_f = 0.0f;
            if (glm::length(eta) < EPSILON && intIOR > EPSILON) {
                eta_f = intIOR / extIOR;
            }
            glm::vec3 k = intersection.material()->k;

            // 计算 GGX 分布 D, 几何遮蔽 G 和 菲涅尔项 F
            float D = GGX_D(halfDir, normal, alpha);
            float G = GGX_G(lightDir, viewDir, normal, halfDir);
            glm::vec3 F = glm::vec3(0.0f);
            float cosTheta = glm::dot(viewDir, halfDir);  // 计算视线方向与半程向量之间的夹角余弦
            if (intersection.material()->type == MaterialType::RoughConductor) {
                // k != 0, 对导体材质（金属）的反射
                F = computeFresnelConductor(cosTheta, eta, k);
            }
            else {
                // k = 0, 对塑料，陶瓷的反射
                float F0 = 0.0;  //all the eta = 2/3
                float F_f = GGX_F(viewDir, halfDir, F0);
                F = glm::vec3(F_f, F_f, F_f);
            }

            // 计算有一定粗糙度的，ggx分布的镜面反射光照
            auto spec = (D * G * F) / (4.0f * glm::dot(lightDir, normal) * glm::dot(viewDir, normal));

            specularColor += specularReflect * spec * (1.0f / (lightDistance * lightDistance));
        }
    }
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
glm::vec3 PathTracer::computeRefractionLighting(Intersection& intersection, BVH& bvh, const Scene& scene, const Ray& ray) {
    glm::vec3 refractionColor(0.0f, 0.0f, 0.0f);  
    glm::vec3 viewDir = glm::normalize(ray.direction); 
    glm::vec3 normal = intersection.normal(); 

    // Step 1: Handle two-sided material
    if (intersection.material()->twoSided && glm::dot(ray.direction, normal) > 0) {   //back face
        normal = -normal;
    }
    for (const auto& light : scene.lights) {
        glm::vec3 lightDir = light.position - intersection.point();  
        float lightDistance = glm::length(lightDir);
        float lightIntensity = light.intensity / (lightDistance * lightDistance);
        lightDir = glm::normalize(lightDir);

        // Check if the point is in shadow (if the light is blocked by other objects)
        Ray shadowRay = { intersection.point(), lightDir };
        Intersection shadowIntersection = Intersection();
        float t = 1e6;
        bool hit = bvh.intersect(shadowRay, shadowIntersection, t);
        if (hit) {
            continue; // Point is in shadow, skip this light
        }

        float intIOR = intersection.material()->int_ior;
        float extIOR = intersection.material()->ext_ior;
        float eta_f = intIOR / extIOR;
        float F0 = 0.04;

        glm::vec3 halfDir = glm::normalize(lightDir + viewDir);
        float fresnelTerm = GGX_F(viewDir, halfDir, F0);

        float diff = std::max(0.0f, glm::dot(intersection.normal(), lightDir));
        refractionColor += (1 - fresnelTerm) * lightIntensity;  //intensity

    }
    return refractionColor;
}


static std::mt19937 generator(std::random_device{}());
std::uniform_real_distribution<float> distribution(0.0, 1.0);

glm::vec3 PathTracer::generateRandomDirection(glm::vec3 normal) {

    float r1 = 2 * M_PI * distribution(generator);  // 随机方位角
    float r2 = distribution(generator);             // 随机半径平方
    float r2s = std::sqrt(r2);                      // 随机半径

    // 计算基于法向量的坐标系
    glm::vec3 w = glm::normalize(normal);                                  // 确保w为单位向量
    glm::vec3 u = (fabs(w.x) > 0.1 ? glm::vec3(0, 1, 0) : glm::normalize(glm::cross(glm::vec3(1, 0, 0), w)));
    glm::vec3 v = glm::cross(w, u);

    // 计算随机方向向量
    glm::vec3 d = (u * std::cos(r1) * r2s + v * std::sin(r1) * r2s + glm::normalize(w * std::sqrt(1 - r2)));

    return glm::normalize(d);
}


// 追踪路径
glm::vec3 PathTracer::tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount) {
    if (bounceCount >= MAX_BOUNCES)
        return glm::vec3(0.0f);
    glm::vec3 result_color = glm::vec3(0.0f);
    Intersection intersection_scene = Intersection();
    float t = 1e6;
    bool intersect_scene = bvh.intersect(ray, intersection_scene, t);
    Intersection intersection_light = Intersection();
    Light light_intersect = { glm::vec3(0.0f), glm::vec3(0.0f), 0.0f };
    std::vector<Light> lights = scene.lights;
    bool intersect_light = intersectLight(ray, lights, light_intersect, intersection_light);
    if (intersect_scene || intersect_light)
        std::cout << "Intersect." << std::endl;
    else {
        std::cout << "No intersection." << std::endl;
    }
    if ((intersect_light && !intersect_scene) || (intersect_light && intersect_scene && intersection_light.t() < intersection_scene.t())) {
        //light intersection
        glm::vec3 color;
        color = light_intersect.color * light_intersect.intensity;
        result_color = glm::vec3(light_intersect.color) * light_intersect.intensity;
        std::cout << "Intersct with light." <<std::endl;
        return result_color;
    }
    if ((intersect_scene && !intersect_light) || (intersect_light && intersect_scene && intersection_scene.t() < intersection_light.t())) {
        //object intersection
        const Material* material_intersect = intersection_scene.material();
        //std::cout << material_intersect->specularColor.r_ << material_intersect->specularColor.g_ << material_intersect->specularColor.b_ << std::endl;
        //std::cout << material_intersect->diffuseColor.r_ << material_intersect->diffuseColor.g_ << material_intersect->diffuseColor.b_ << std::endl;
        glm::vec3 position_new = intersection_scene.point();
        glm::vec3 specular_color = material_intersect->specularReflect;
        glm::vec3 diffuse_color = material_intersect->diffuseReflect;
        float intIOR = material_intersect->int_ior;
        float extIOR = material_intersect->ext_ior;

        glm::vec3 normal = intersection_scene.normal();

        // Step 1: Handle two-sided material
        if (material_intersect->twoSided && glm::dot(ray.direction, normal) > 0) {   //back face
            normal = -normal;
        }

        if (glm::length(diffuse_color) > EPSILON) {
            // exists diffuse 
            result_color += computeDiffuseLighting(intersection_scene, bvh, scene);
        }

        if (glm::length(specular_color) > EPSILON) {
            // exists reflect
            // conductor material 这一层调用 贡献为0， 递归调用后路径追踪到不是conductor material时才有贡献
            result_color += computeSpecularLighting(intersection_scene, bvh, scene, ray);
        }

        if (intIOR > EPSILON && extIOR > EPSILON) {
            // exists refraction
            result_color += computeRefractionLighting(intersection_scene, bvh, scene, ray);
        }

        int bounceCount_new = bounceCount + 1;

        if (glm::length(specular_color) > EPSILON) {
            glm::vec3 direction_new = ray.direction - 2 * glm::dot(ray.direction, normal) * normal;
            Ray ray_new = { position_new, direction_new };

            std::cout << "Intersect with specular object." << std::endl;

            result_color += tracePath(ray_new, scene, bvh, bounceCount_new);
        }
        if (glm::length(diffuse_color) > EPSILON) {
            glm::vec3 direction_new = generateRandomDirection(normal);
            Ray ray_new = { position_new, direction_new };

            std::cout << "Intersect with diffuse object." << std::endl;

            result_color += tracePath(ray_new, scene, bvh, bounceCount_new);

        }
        if (intIOR > EPSILON && extIOR > EPSILON) {
            glm::vec3 refractDir = refractDirection(ray.direction, normal, intIOR, extIOR);
            Ray refractRay = { position_new, refractDir };

            result_color += tracePath(refractRay, scene, bvh, bounceCount_new);
        }
    } 
    return glm::vec3(0.0f, 0.0f, 0.0f);
}

//generate a sample
glm::vec3 generateSample(const Camera& camera, int x, int y, int width, int height) {
    float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
    float scale = tan(glm::radians(camera.fov * 0.5f));

    float u = (2.0f * ((x + distribution(generator)) / static_cast<float>(width)) - 1.0f) * aspect_ratio * scale;
    float v = (1.0f - 2.0f * ((y + distribution(generator)) / static_cast<float>(height))) * scale;

    glm::vec3 forward = glm::vec3(0.0, 0.0, -1.0);
    glm::vec3 right = glm::vec3(1.0, 0.0, 0.0);
    glm::vec3 up = glm::vec3(0.0, 1.0, 0.0);

    glm::vec3 sample_direction = glm::normalize(camera.rotationMatrix * (forward + u * right + v * up));
    sample_direction = camera.rotationMatrix * sample_direction;
    return sample_direction;
}

// 主路径追踪函数
void PathTracer::render(const Scene& scene, const Camera& camera, BVH& bvh, int width, int height, int samplesPerPixel) {
    std::vector < glm::vec3 > framebuffer(width * height, glm::vec3(0.0f, 0.0f, 0.0f));
    glm::vec3 camPos = camera.position;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            glm::vec3 pixel_radiance(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < samplesPerPixel; ++i) {
                glm::vec3 sample_position = generateSample(camera, x, y, width, height); // 调用完成的生成样本位置函数
                //std::cout << "sample_position: " << sample_position[0] << " " << sample_position[1] << " " << sample_position[2] << std::endl;
                Ray ray = { camPos, sample_position};
                glm::vec3 color_mid = tracePath(ray, scene, bvh, 0);
                pixel_radiance = pixel_radiance + color_mid * (1.0f / (float)samplesPerPixel);
            }

            framebuffer[y * width + x] = pixel_radiance;
            std::cout << "Color: " << pixel_radiance[0] << " " << pixel_radiance[1] << " " << pixel_radiance[2] << std::endl;
            std::cout << "Finish rendering pixel (" << x << ", " << y << ")." << std::endl;
        }
    }

    // 输出图像到文件（如.ppm格式）
    std::ofstream outFile("output.ppm");
    outFile << "P3\n" << width << " " << height << "\n255\n";
    for (auto& color : framebuffer) {
        outFile << static_cast<int>(std::clamp(color[0] * 255.0f, 0.0f, 255.0f)) << " ";
        outFile << static_cast<int>(std::clamp(color[1] * 255.0f, 0.0f, 255.0f)) << " ";
        outFile << static_cast<int>(std::clamp(color[2] * 255.0f, 0.0f, 255.0f)) << "\n";
    }
    outFile.close();
}
