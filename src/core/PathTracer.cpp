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

bool rayIntersectsTriangle(const Ray& ray, const Triangle& triangle, float& t, glm::vec3& normal) {
    glm::vec3 rayOrigin = ray.position;
    glm::vec3 rayDir = glm::normalize(ray.direction);
    glm::vec3 e1 = triangle.v1 - triangle.v0;
    glm::vec3 e2= triangle.v2 - triangle.v0;
    glm::vec3 n = glm::normalize( glm::cross(e1,e2) );
    
    float denominator = glm::dot(rayDir, n);
    if (denominator > -EPSILON && denominator < EPSILON)
        return false;

    float criteria = dot(triangle.v0 - rayOrigin, n) / denominator;
    if (criteria < 0.0)
        return false;
    
    glm::vec3 s = rayOrigin - triangle.v0;
    glm::vec3 s1 = glm::cross(rayDir, e2);
    glm::vec3 s2 = glm::cross(s, e1);

    float invDenom = 1.0 / glm::dot(s1, e1);
    float b1 = dot(s1, s) * invDenom;
    float b2 = dot(s2, rayDir) * invDenom;
    t = dot(s2, e2) * invDenom;

    if (t > 1e-6 && b1 >= 0.0 && b2 >= 0.0 && b1 + b2 <= 1.0) {
        glm::vec3 position = rayOrigin + criteria * rayDir;
        normal = n;
        return true;
    }
    else {
        return false;
    }
}

bool intersectScene(Ray& ray, const Scene& scene, Intersection& intersection) {
	bool hit = false;
	float closestHit = std::numeric_limits<float>::max();
	for (const auto& model : scene.getModels()) {
		for (auto& mesh : model->meshes) {
            for (size_t i = 0; i < mesh.indices.size() / 3; ++i) {
                glm::vec3& v0 = mesh.vertices[mesh.indices[i * 3]];
                glm::vec3& v1 = mesh.vertices[mesh.indices[i * 3 + 1]];
                glm::vec3& v2 = mesh.vertices[mesh.indices[i * 3 + 2]];


                Triangle triangle(v0, v1, v2);

                float t = 0.0f;
                glm::vec3 point = glm::vec3(0.0f);
                glm::vec3 normal = glm::vec3(0.0f);

                if (rayIntersectsTriangle(ray, triangle, t, normal) && t < closestHit) {
                    hit = true;
                    closestHit = t;
                    point = ray.position + t * ray.direction;
                    Material* p = &mesh.material_;

                    // 更新相交信息
                    intersection.set(t, point, normal, p);
                }
            }
		}
	}
	return hit;
}

// 漫反射光照
Color PathTracer::computeDiffuseLighting(Intersection& intersection, const Scene& scene) {
    Color diffuseColor(0.0f, 0.0f, 0.0f);  

    for (const auto& light : scene.getLights()) {
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
    for (const auto& light : scene.getLights()) {
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
Color PathTracer::tracePath(Ray ray, const Scene& scene, int bounceCount) {
    if (bounceCount > MAX_BOUNCES) return Color(0.0f, 0.0f, 0.0f); // 达到反弹次数限制

    Color result_color = Color();
    Intersection intersection_scene = Intersection();
    bool intersect_scene = intersectScene(ray, scene, intersection_scene);
    Intersection intersection_light = Intersection();
    Light light_intersect = { glm::vec3(0.0f), glm::vec3(0.0f), 0.0f };
    std::vector<Light> lights = scene.getLights();
    bool intersect_light = intersectLight(ray, lights, light_intersect, intersection_light);
    if (intersect_scene || intersect_light)
        std::cout << "Intersect." << std::endl;
    else {
        std::cout << "No intersection." << std::endl;
        return Color(0.0f, 0.0f, 0.0f);
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
        glm::vec3 position_new = intersection_scene.point();
        Color specular_color = material_intersect->specularColor;
        Color diffuse_color = material_intersect->diffuseColor;
        if (glm::length(glm::vec3(specular_color.r_, specular_color.g_, specular_color.b_)) > EPSILON) {
            glm::vec3 direction_new = ray.direction - 2 * glm::dot(ray.direction, intersection_scene.normal()) * intersection_scene.normal();
            Ray ray_new = { position_new, direction_new };
            int bounceCount_new = bounceCount + 1;
            result_color = tracePath(ray_new, scene, bounceCount_new);
            std::cout << "Itersect with specular object." << std::endl;
            return result_color;
        }
        else  {
            glm::vec3 direction_new = generateRandomDirection(intersection_scene.normal());
            Ray ray_new = { position_new, direction_new };
            int bounceCount_new = bounceCount + 1;
            result_color = tracePath(ray_new, scene, bounceCount_new);
            std::cout << "Itersect with diffuse object." << std::endl;
            return result_color;
        }
    }   
}

//generate a sample
glm::vec3 generateSample(const Camera& camera, int x, int y, int width, int height) {
    float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
    float scale = tan(glm::radians(camera.fov * 0.5f));

    float u = (2.0f * ((x + distribution(generator)) / static_cast<float>(width)) - 1.0f) * aspect_ratio * scale;
    float v = (1.0f - 2.0f * ((y + distribution(generator)) / static_cast<float>(height))) * scale;

    glm::vec3 forward = glm::normalize(camera.direction);
    glm::vec3 right = glm::normalize(glm::cross(forward, camera.up));
    glm::vec3 up = glm::normalize(glm::cross(right, forward));

    glm::vec3 sample_direction = glm::normalize(forward + u * right + v * up);
    return sample_direction;
}

// 主路径追踪函数
void PathTracer::render(const Scene& scene, const Camera& camera, int width, int height, int samplesPerPixel) {
    std::vector<Color> framebuffer(width * height, Color(0.0f, 0.0f, 0.0f));
    glm::vec3 camPos = camera.position;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Color pixel_radiance(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < samplesPerPixel; ++i) {
                glm::vec3 sample_position = generateSample(camera, x, y, width, height); // 调用完成的生成样本位置函数
                Ray ray = { camPos, sample_position};
                Color color_mid = tracePath(ray, scene, 0);
                pixel_radiance = pixel_radiance + color_mid * (1.0f / (float)samplesPerPixel);
            }

            framebuffer[y * width + x] = pixel_radiance;
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
