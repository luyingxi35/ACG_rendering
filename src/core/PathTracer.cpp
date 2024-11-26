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
    Material* material = nullptr;
    float closestHit = std::numeric_limits<float>::max();
    for (const auto& light : lights) {
        float t = glm::length(light.position - pos);
        if (glm::dot(light.position - pos, dir) > 0 && (1 - glm::dot(light.position - pos, dir)) < EPSILON && t < closestHit) {
            hit = true;
            closestHit = t;
            result_light = light;
            intersection.set(t, glm::vec3(0.0f), glm::vec3(0.0f), material);
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
        Material* material = & model->material;
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

                    // �����ཻ��Ϣ
                    intersection.set(t, point, normal, material);
                }
            }
		}
	}
	return hit;
}

// ���������
glm::vec3 PathTracer::computeDiffuseLighting(Intersection& intersection, const Scene& scene) {
    glm::vec3 diffuseColor(0.0f, 0.0f, 0.0f);

    for (const auto& light : scene.lights) {
        glm::vec3 lightDir = light.position - intersection.point();  // ���߷���
        float lightDistance = glm::length(lightDir);
		float lightIntensity = light.intensity / (lightDistance * lightDistance);
		lightDir = glm::normalize(lightDir);

        // Check if the point is in shadow (if the light is blocked by other objects)
        Ray shadowRay = { intersection.point(), lightDir };
        Intersection shadowIntersection = Intersection();
		bool hit = intersectScene(shadowRay, scene, shadowIntersection);
        if (hit) {
            continue; // Point is in shadow, skip this light
        }

        float diff = std::max(0.0f, glm::dot(intersection.normal(), lightDir));
        diffuseColor += intersection.material()->diffuseReflect * diff * lightIntensity;  //��intensity
        
    }

    return diffuseColor;
}


// GGX�ֲ�ģ��
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
    
// ���㵼����ʵķ�������  ÿ����������, eta F_0 ��Ҫ�� glm::vec3
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


// ���㾵�淴�����
glm::vec3 PathTracer::computeSpecularLighting(Intersection& intersection, const Scene& scene, const Ray& ray) {
    glm::vec3 specularColor(0.0f, 0.0f, 0.0f);  // ��ʼ�ľ��淴����ɫ
    glm::vec3 viewDir = glm::normalize(ray.direction);  // ���߷���
	glm::vec3 normal = intersection.normal();  // ���߷���

    // Step 1: Handle two-sided material
    if (intersection.material()->twoSided && glm::dot(ray.direction, normal) > 0) {   //back face
        normal = -normal;
    }

	// ��������ǵ��壬���㵼��ľ��淴��
    if (intersection.material()->type == MaterialType::Conductor) {
        for (const auto& light : scene.lights) {
            glm::vec3 lightDir = light.position - intersection.point();  // ���߷��� 
            float lightDistance = glm::length(lightDir);
            lightDir = glm::normalize(lightDir);

            // Check if the point is in shadow (if the light is blocked by other objects)
            Ray shadowRay = { intersection.point(), lightDir };
            Intersection shadowIntersection = Intersection();
            bool hit = intersectScene(shadowRay, scene, shadowIntersection);
            if (hit) {
                continue; // Point is in shadow, skip this light
            }

            // �����������淴��ķ��䷽��
            glm::vec3 reflectDir = glm::normalize(glm::reflect(-lightDir, normal));
            float lightIntensity = light.intensity;
            float spec = std::pow(std::max(0.0f, glm::dot(reflectDir, lightDir)), intersection.material()->alpha);
            
			// ����������� nonlinear
            //float cosTheta = glm::dot(viewDir, normal);
            //float fresnel = pow(1.0f - cosTheta, 5);  //(1-cosTheta)^5

            specularColor += intersection.material()->specularReflect * spec * (1 / (lightDistance * lightDistance)) * lightIntensity;
        }
    }else{
        // ��ȡ������Ϣ
        float alpha = intersection.material()->alpha;
        glm::vec3 specularReflect = intersection.material()->specularReflect;

        // �������й�Դ������ÿ����Դ�Ծ��淴��Ĺ���
        for (const auto& light : scene.lights) {
            glm::vec3 lightDir = light.position - intersection.point();  // ���߷��� 
            float lightDistance = glm::length(lightDir);
            lightDir = glm::normalize(lightDir);

            // Check if the point is in shadow (if the light is blocked by other objects)
            Ray shadowRay = { intersection.point(), lightDir };
            Intersection shadowIntersection = Intersection();
            bool hit = intersectScene(shadowRay, scene, shadowIntersection);
            if (hit) {
                continue; // Point is in shadow, skip this light
            }

            // ���������� h
            glm::vec3 halfDir = glm::normalize(lightDir + viewDir);

            // �дֲڶȵľ��淴�� or �����Բ���
            float intIOR = intersection.material()->int_ior;
            float extIOR = intersection.material()->ext_ior;
            glm::vec3 eta = intersection.material()->eta;
            float eta_f = 0.0f;
            if (glm::length(eta) < EPSILON && intIOR > EPSILON) {
                eta_f = intIOR / extIOR;
            }
            glm::vec3 k = intersection.material()->k;

            // ���� GGX �ֲ� D, �����ڱ� G �� �������� F
            float D = GGX_D(halfDir, normal, alpha);
            float G = GGX_G(lightDir, viewDir, normal, halfDir);
            glm::vec3 F = glm::vec3 (0.0f);
            float cosTheta = glm::dot(viewDir, halfDir);  // �������߷�����������֮��ļн�����
            if (intersection.material()->type == MaterialType::RoughConductor) {
                // k != 0, �Ե�����ʣ��������ķ���
                F = computeFresnelConductor(cosTheta, eta, k);
            }
            else {
                // k = 0, �����ϣ��մɵķ���
                float F0 = 0.0;  //all the eta = 2/3
				float F_f = GGX_F(viewDir, halfDir, F0);
                F = glm::vec3(F_f, F_f, F_f);
            }

            // ������һ���ֲڶȵģ�ggx�ֲ��ľ��淴�����
            auto spec = (D * G * F) / (4.0f * glm::dot(lightDir, normal) * glm::dot(viewDir, normal));

            specularColor += specularReflect * spec * (1.0f / (lightDistance * lightDistance));
        }
	}
    return specularColor;
}

// ����������߷���
glm::vec3 PathTracer::refractDirection(const glm::vec3& incident, const glm::vec3& normal, float ext_ior, float int_ior) {
    float eta = ext_ior / int_ior; // �������� / ��������

    // ��������ǵ�����ֵ
    float cosi = glm::clamp(glm::dot(incident, normal), -1.0f, 1.0f);

    // �ж��Ƿ���ȫ�ڷ���
    float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
    if (k < 0.0f) {
        return glm::vec3(0.0f); // Total internal reflection, no refraction
    }
    else {
        return eta * incident - (eta * cosi + sqrt(k)) * normal;
    }
}
//�����������
glm::vec3 PathTracer::computeRefractionLighting(Intersection& intersection, const Scene& scene, const Ray& ray) {
    glm::vec3 refractionColor(0.0f, 0.0f, 0.0f);  // ��ʼ��������ɫ
    glm::vec3 viewDir = glm::normalize(ray.direction);  // ���߷���
    glm::vec3 normal = intersection.normal();  // ���߷���

    // Step 1: Handle two-sided material
    if (intersection.material()->twoSided && glm::dot(ray.direction, normal) > 0) {   //back face
        normal = -normal;
    }
    for (const auto& light : scene.lights) {
        glm::vec3 lightDir = light.position - intersection.point();  // ���߷���
        float lightDistance = glm::length(lightDir);
        float lightIntensity = light.intensity / (lightDistance * lightDistance);
        lightDir = glm::normalize(lightDir);

        // Check if the point is in shadow (if the light is blocked by other objects)
        Ray shadowRay = { intersection.point(), lightDir };
        Intersection shadowIntersection = Intersection();
        bool hit = intersectScene(shadowRay, scene, shadowIntersection);
        if (hit) {
            continue; // Point is in shadow, skip this light
        }

        // ����������������Ĺ���
        float intIOR = intersection.material()->int_ior;
        float extIOR = intersection.material()->ext_ior;
        float eta_f = intIOR / extIOR;
        float F0 = 0.04;
        // ���������� h
        glm::vec3 halfDir = glm::normalize(lightDir + viewDir);
        float fresnelTerm = GGX_F(viewDir, halfDir, F0);

        float diff = std::max(0.0f, glm::dot(intersection.normal(), lightDir));
        refractionColor += (1-fresnelTerm) * lightIntensity;  //��intensity

    }
    return refractionColor;
}

static std::mt19937 generator(std::random_device{}());
std::uniform_real_distribution<float> distribution(0.0, 1.0);

glm::vec3 PathTracer::generateRandomDirection(glm::vec3 normal) {

    float r1 = 2 * M_PI * distribution(generator);  // �����λ��
    float r2 = distribution(generator);             // ����뾶ƽ��
    float r2s = std::sqrt(r2);                      // ����뾶

    // ������ڷ�����������ϵ
    glm::vec3 w = glm::normalize(normal);                                  // ȷ��wΪ��λ����
    glm::vec3 u = (fabs(w.x) > 0.1 ? glm::vec3(0, 1, 0) : glm::normalize(glm::cross(glm::vec3(1, 0, 0), w)));
    glm::vec3 v = glm::cross(w, u);

    // ���������������
    glm::vec3 d = (u * std::cos(r1) * r2s + v * std::sin(r1) * r2s + glm::normalize(w * std::sqrt(1 - r2)));

    return glm::normalize(d);
}


// ׷��·��
glm::vec3 PathTracer::tracePath(Ray ray, const Scene& scene, int bounceCount) {
    if (bounceCount >= MAX_BOUNCES) {
        return glm::vec3(0.0f);  // ���ر�����ɫ
    }
    glm::vec3 result_color = glm::vec3(0.0f);
	Intersection intersection_scene = Intersection();  //�볡���ཻ�Ľ���
    bool intersect_scene = intersectScene(ray, scene, intersection_scene);  //�Ƿ��볡�������壩��
	Intersection intersection_light = Intersection();  //���Դ�ཻ�Ľ���
	Light light_intersect = { glm::vec3(0.0f), glm::vec3(0.0f), 0.0f };  //�����洢�������ཻ�Ĺ�Դ
	std::vector<Light> lights = scene.lights;  //��Դ
	bool intersect_light = intersectLight(ray, lights, light_intersect, intersection_light);  //�Ƿ����Դ��
    /*if (intersect_scene || intersect_light)
        std::cout << "Intersect." << std::endl;
    else {
        std::cout << "No intersection." << std::endl;
    }*/
    if ((intersect_light && !intersect_scene) || (intersect_light && intersect_scene && intersection_light.t() < intersection_scene.t())) {
        //light intersection
        result_color = glm::vec3(light_intersect.color) * light_intersect.intensity;
        std::cout << "Intersct with light." <<std::endl;
        return result_color;
    }
    if ((intersect_scene && !intersect_light) || (intersect_light && intersect_scene && intersection_scene.t() < intersection_light.t())) {
        //object intersection
        Material* material_intersect = intersection_scene.material();
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
            // ���������������
            result_color += computeDiffuseLighting(intersection_scene, scene);
        }

        if (glm::length(specular_color) > EPSILON) {  
			// ��������о��淴��
			// conductor material ��һ����� ����Ϊ0�� �ݹ���ú�·��׷�ٵ�����conductor materialʱ���й���
            result_color += computeSpecularLighting(intersection_scene, scene, ray);
        }

        if (intIOR > EPSILON && extIOR > EPSILON) {
            // �������������
            result_color += computeRefractionLighting(intersection_scene, scene, ray);
        }

        int bounceCount_new = bounceCount + 1;

        if (glm::length(specular_color) > EPSILON) {
			// ��������о��淴�䣬�ݹ�׷�ٷ������
            glm::vec3 direction_new = ray.direction - 2 * glm::dot(ray.direction, normal) * normal;
            Ray ray_new = { position_new, direction_new };

            std::cout << "Intersect with specular object." << std::endl;
            
            result_color += tracePath(ray_new, scene, bounceCount_new);
        }
        if (glm::length(diffuse_color) > EPSILON) {
			// ��������������䣬 �ݹ�׷�����������
            glm::vec3 direction_new = generateRandomDirection(normal);
            Ray ray_new = { position_new, direction_new };
            
            std::cout << "Intersect with diffuse object." << std::endl;

            result_color += tracePath(ray_new, scene, bounceCount_new);
            
        }
        if (intIOR > EPSILON && extIOR > EPSILON) {
			// ���������������Ϊ���ݹ�׷���������
            glm::vec3 refractDir = refractDirection(ray.direction, normal, intIOR, extIOR);
            Ray refractRay = { position_new, refractDir };
            
            // �������������չ��ף���Ȩ��  // �����������ܺ�Ӧ��Ϊ1.0
            result_color += tracePath(refractRay, scene, bounceCount_new);
        }
    } 
    return result_color;
}

//generate a sample
glm::vec3 generateSample(const Camera& camera, int x, int y, int width, int height) {
    float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
    float scale = tan(glm::radians(camera.fov * 0.5f));

    float u = (2.0f * ((x + distribution(generator)) / static_cast<float>(width)) - 1.0f) * aspect_ratio * scale;
    float v = (1.0f - 2.0f * ((y + distribution(generator)) / static_cast<float>(height))) * scale;

    glm::vec3 forward(0.0, 0.0, -1.0);
    glm::vec3 right(1.0, 0.0, 0.0);
    glm::vec3 up(0.0, 1.0, 0.0);

    glm::vec3 sample_direction = glm::normalize(camera.rotationMatrix * (forward + u * right + v * up));
    sample_direction = camera.rotationMatrix * sample_direction;
    return sample_direction;
}

// ��·��׷�ٺ���
void PathTracer::render(const Scene& scene, const Camera& camera, int width, int height, int samplesPerPixel) {
    std::vector<glm::vec3> framebuffer(width * height, glm::vec3(0.0f, 0.0f, 0.0f));
    glm::vec3 camPos = camera.position;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            glm::vec3 pixel_radiance(0.0f, 0.0f, 0.0f);

            for (int i = 0; i < samplesPerPixel; ++i) {
                glm::vec3 sample_position = generateSample(camera, x, y, width, height); // ������ɵ���������λ�ú���
                Ray ray = { camPos, sample_position};
                glm::vec3 color_mid = tracePath(ray, scene, 0);
                pixel_radiance = pixel_radiance + color_mid * (1.0f / (float)samplesPerPixel);
            }

            framebuffer[y * width + x] = pixel_radiance;
            std::cout << "Finish rendering pixel (" << x << ", " << y << ")." << std::endl;
        }
    }

    // ���ͼ���ļ�����.ppm��ʽ��
    std::ofstream outFile("output.ppm");
    outFile << "P3\n" << width << " " << height << "\n255\n";
    for (auto& color : framebuffer) {
        outFile << static_cast<int>(glm::clamp(color[0] * 255.0f, 0.0f, 255.0f)) << " ";
        outFile << static_cast<int>(glm::clamp(color[1] * 255.0f, 0.0f, 255.0f)) << " ";
        outFile << static_cast<int>(glm::clamp(color[2] * 255.0f, 0.0f, 255.0f)) << "\n";
    }
    outFile.close();
}
