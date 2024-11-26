#include "PathTracer.h"
#define EPSILON 1e-6
#define M_PI 3.1415926535

const int MAX_BOUNCES = 5;
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> dis(0.0f, 1.0f);

//point light
bool intersectLight(const Ray& ray, const std::vector<Light> lights, Light& result_light, Intersection& intersection) {
    bool hit = false;
    glm::vec3 pos = ray.position;
    glm::vec3 dir = glm::normalize(ray.direction);
    Material material = { 0,Color(), Color(), 0.0,0.0 };
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
Color PathTracer::computeDiffuseLighting(Intersection& intersection, const Scene& scene) {
    Color diffuseColor(0.0f, 0.0f, 0.0f);  

    for (const auto& light : scene.lights) {
        glm::vec3 lightDir = light.position- intersection.point();  // 光线方向
        float lightDistance = glm::length(lightDir);
        lightDir = glm::normalize(lightDir);

        float diff = std::max(0.0f, glm::dot(intersection.normal(), lightDir));

        diffuseColor = diffuseColor + intersection.material()->diffuseColor * diff * (1 / (lightDistance * lightDistance)); 
    }

    return diffuseColor;
}


// 计算镜面反射光照
Color PathTracer::computeSpecularLighting(Intersection& intersection, const Scene& scene) {
    Color specularColor(0.0f, 0.0f, 0.0f);  // 初始的镜面反射颜色

    // 遍历所有光源，计算每个光源对镜面反射的贡献
    for (const auto& light : scene.lights) {
        glm::vec3 lightDir = light.position - intersection.point();  // 光线方向
        float lightDistance = glm::length(lightDir);
        lightDir = glm::normalize(lightDir);

        // 计算镜面反射的反射方向
        glm::vec3 reflectDir = glm::normalize(glm::reflect(-lightDir, intersection.normal()));
        float spec = std::pow(std::max(0.0f, glm::dot(reflectDir, lightDir)), intersection.material()->shineness);

        specularColor = specularColor + intersection.material()->specularColor * spec * (1 / (lightDistance * lightDistance)); 
    }

    return specularColor;
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
Color PathTracer::tracePath(Ray ray, const Scene& scene, BVH& bvh, int bounceCount) {
    Color result_color = Color();
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
        result_color = Color(color[0], color[1], color[2]);
        std::cout << "Intersct with light." <<std::endl;
        return result_color;
    }
    if ((intersect_scene && !intersect_light) || (intersect_light && intersect_scene && intersection_scene.t() < intersection_light.t())) {
        //object intersection
        const Material* material_intersect = intersection_scene.material();
        //std::cout << material_intersect->specularColor.r_ << material_intersect->specularColor.g_ << material_intersect->specularColor.b_ << std::endl;
        //std::cout << material_intersect->diffuseColor.r_ << material_intersect->diffuseColor.g_ << material_intersect->diffuseColor.b_ << std::endl;
        glm::vec3 position_new = intersection_scene.point();
        Color specular_color = material_intersect->specularColor;
        Color diffuse_color = material_intersect->diffuseColor;
        if (glm::length(glm::vec3(specular_color.r_, specular_color.g_, specular_color.b_)) > EPSILON) {
            glm::vec3 direction_new = ray.direction - 2 * glm::dot(ray.direction, intersection_scene.normal()) * intersection_scene.normal();
            Ray ray_new = { position_new, direction_new };
            int bounceCount_new = bounceCount + 1;
            std::cout << "Itersect with specular object." << std::endl;
            if (bounceCount_new > MAX_BOUNCES) {
                result_color = material_intersect->specularColor;
                return result_color;
            }
            result_color = tracePath(ray_new, scene, bvh, bounceCount_new);
            return result_color;
        }
        else  {
            glm::vec3 direction_new = generateRandomDirection(intersection_scene.normal());
            Ray ray_new = { position_new, direction_new };
            int bounceCount_new = bounceCount + 1;
            std::cout << "Itersect with diffuse object." << std::endl;
            if (bounceCount_new > MAX_BOUNCES) {
                result_color = material_intersect->diffuseColor;
                return result_color;
            }
            result_color = tracePath(ray_new, scene, bvh, bounceCount_new);
            return result_color;
        }
    } 
    return Color(0.0f, 0.0f, 0.0f);
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
    std::vector<Color> framebuffer(width * height, Color(0.0f, 0.0f, 0.0f));
    glm::vec3 camPos = camera.position;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Color pixel_radiance(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < samplesPerPixel; ++i) {
                glm::vec3 sample_position = generateSample(camera, x, y, width, height); // 调用完成的生成样本位置函数
                std::cout << "sample_position: " << sample_position[0] << " " << sample_position[1] << " " << sample_position[2] << std::endl;
                Ray ray = { camPos, sample_position};
                Color color_mid = tracePath(ray, scene, bvh, 0);
                pixel_radiance = pixel_radiance + color_mid * (1.0f / (float)samplesPerPixel);
            }

            framebuffer[y * width + x] = pixel_radiance;
            std::cout << "Color: " << pixel_radiance.r() << " " << pixel_radiance.g() << " " << pixel_radiance.b() << std::endl;
            std::cout << "Finish rendering pixel (" << x << ", " << y << ")." << std::endl;
        }
    }

    // 输出图像到文件（如.ppm格式）
    std::ofstream outFile("output.ppm");
    outFile << "P3\n" << width << " " << height << "\n255\n";
    for (auto& color : framebuffer) {
        outFile << static_cast<int>(std::clamp(color.r() * 255.0f, 0.0f, 255.0f)) << " ";
        outFile << static_cast<int>(std::clamp(color.g() * 255.0f, 0.0f, 255.0f)) << " ";
        outFile << static_cast<int>(std::clamp(color.b() * 255.0f, 0.0f, 255.0f)) << "\n";
    }
    outFile.close();
}
