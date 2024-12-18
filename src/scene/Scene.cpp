#include "Scene.h"

namespace fs = std::filesystem;

Scene::Scene() {
}

Scene::~Scene() {}

void Scene::addModel(std::shared_ptr<Model> model) {
    models.push_back(model);
}



// 提取颜色
glm::vec3 extractColor(const std::string& value) {
    std::istringstream ss(value);
    float r, g, b;
    char comma; // 跳过逗号
    ss >> r >> comma >> g >> comma >> b;
    return glm::vec3(r, g, b);
}

float extractFloat(pugi::xml_node bsdf, const std::string& name) {
    for (auto node : bsdf.children("float")) {
        if (node && std::string(node.attribute("name").as_string()) == name) {
            return node.attribute("value").as_float();
        }
    }
    return 0.0f;  // 默认值
}

glm::vec3 extractRGB(pugi::xml_node bsdf, const std::string& name) {
    for (auto node : bsdf.children("rgb")) {
        if (node && std::string(node.attribute("name").as_string()) == name) {
            return extractColor(node.attribute("value").as_string());
        }
    }
    return glm::vec3(0.0f);  // 默认值
}

Material Scene::extractMaterialFromBSDF(pugi::xml_node bsdf) {
    Material material;

    std::string type = bsdf.attribute("type").as_string();
    if (type == "twosided") {
        material.twoSided = true;
        type = bsdf.child("bsdf").attribute("type").as_string();
    }

    if (type == "roughdielectric") {
        material.type = MaterialType::RoughDielectric;
        material.alpha = extractFloat(bsdf, "alpha");
        material.int_ior = extractFloat(bsdf, "int_ior");
        material.ext_ior = extractFloat(bsdf, "ext_ior");
    }
    else if (type == "conductor") {
        material.type = MaterialType::Conductor;
		material.specularReflect = glm::vec3(1.0f);
    }
    else if (type == "roughconductor") {
        material.type = MaterialType::RoughConductor;
        material.alpha = extractFloat(bsdf, "alpha");
        //distribution = ggx
        // 提取所有 rgb 属性
        for (auto rgbNode : bsdf.children("rgb")) {
            std::string name = rgbNode.attribute("name").as_string();
            glm::vec3 color = extractColor(rgbNode.attribute("value").as_string());

            if (name == "specular_reflectance") {
                material.specularReflect = color;
            }
            else if (name == "eta") {
                material.eta = color;
            }
            else if (name == "k") {
                material.k = color;
            }
        }
    }
    else if (type == "plastic" || type == "roughplastic") {
        material.type = (type == "plastic") ? MaterialType::Plastic : MaterialType::RoughPlastic;
        if (type == "roughplastic") {
            material.alpha = extractFloat(bsdf, "alpha");
        }
        material.int_ior = extractFloat(bsdf, "int_ior");
        material.ext_ior = extractFloat(bsdf, "ext_ior");
        material.nonlinear = true;
        material.diffuseReflect = extractRGB(bsdf, "diffuse_reflectance");
        
    }
    else if (type == "diffuse") {
        material.type = MaterialType::Diffuse;
        // 提取漫反射材质的反射率
        auto reflectance = bsdf.child("rgb");
        if (reflectance) {
            material.diffuseReflect = extractColor(reflectance.attribute("value").as_string());
        }
    }
    else {
        material.type = MaterialType::Diffuse;  // 默认处理
    }

    return material;
}


void Scene::extractSceneDataFromXML(const std::string& xmlPath, std::vector<Light>& lights, Camera& camera) {
    pugi::xml_document doc;
    if (!doc.load_file(xmlPath.c_str())) {
        std::cerr << "Failed to load XML scene file: " << xmlPath << std::endl;
        return;
    }

    // Extract lights
    for (pugi::xml_node lightNode : doc.child("scene").children("light")) {
        Light light;
        light.position = glm::vec3(
            lightNode.child("position").attribute("x").as_float(),
            lightNode.child("position").attribute("y").as_float(),
            lightNode.child("position").attribute("z").as_float()
        );
        light.color = glm::vec3(
            lightNode.child("color").attribute("r").as_float(),
            lightNode.child("color").attribute("g").as_float(),
            lightNode.child("color").attribute("b").as_float()
        );
        light.intensity = lightNode.child("intensity").text().as_float();
        lights.push_back(light);
    }
    std::cout << "Finish adding lights." << std::endl;
    
    // Extract camera
    pugi::xml_node cameraNode = doc.child("scene").child("sensor");
    camera.fov = cameraNode.child("float").attribute("value").as_float();
    
    camera.rotationMatrix = { {-0.993341, -0.0130485, -0.114467}, {0, 0.993565, -0.11326}, {0.115208, -0.112506, -0.98695}};
    camera.position = glm::vec3(4.44315, 16.934, 49.9102);

    std::cout << "Camera loaded: Position(" << camera.position.x << ", " << camera.position.y << ", " << camera.position.z << ")\n";

    // Extraxt models
    for (auto shapeNode : doc.child("scene").children("shape")) {
        if (std::string(shapeNode.attribute("type").value()) == "obj") {
            std::string path = shapeNode.child("string").attribute("value").as_string();
            std::string filepath = "assets/";
            filepath.append(path);
            std::cout << filepath << std::endl;
            auto model = std::make_shared<Model>();
            model->loadModelFromFile(filepath);


            // Load transformToWorld
            glm::mat4 transform;
            auto matrixNode = shapeNode.child("transform").child("matrix");
            std::string matrixValue = matrixNode.attribute("value").as_string();
            std::cout << matrixValue << std::endl;
            sscanf_s(matrixValue.c_str(), "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
                &transform[0][0], &transform[0][1], &transform[0][2], &transform[0][3],
                &transform[1][0], &transform[1][1], &transform[1][2], &transform[1][3],
                &transform[2][0], &transform[2][1], &transform[2][2], &transform[2][3],
                &transform[3][0], &transform[3][1], &transform[3][2], &transform[3][3]);
            model->transformToWorld = transform;

            // Load material
			auto materialRef = shapeNode.child("ref");
			auto id = materialRef.attribute("id").as_string();
			for (auto bsdf : doc.child("scene").children("bsdf")) {
				if (bsdf.attribute("id").as_string() == id) {
					model->material = extractMaterialFromBSDF(bsdf);
                    break;
				}
			}
			
            addModel(model);
        }
    }
}

const std::vector<std::shared_ptr<Model>>& Scene::getModels() const {
    return models;
}


