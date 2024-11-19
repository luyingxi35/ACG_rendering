#ifndef SCENE_H
#define SCENE_H

#include "Model.h"
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <iostream>
#include <filesystem>
#include <memory>
#include <glm/gtc/matrix_transform.hpp>
#include <pugixml.hpp>
#include <iostream>
#include <sstream>

// 光源类
struct Light {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;
};

// 相机类
struct Camera {
    glm::vec3 position;
    glm::mat3 rotationMatrix;
    float fov;
};

// 光线类
struct Ray {
    glm::vec3 position;
    glm::vec3 direction;
};

class Scene {
public:
    Scene();
    ~Scene();

    void addModel(std::shared_ptr<Model> model);
    void extractSceneDataFromXML(const std::string& xmlPath, std::vector<Light>& lights, Camera& camera);
    const std::vector<std::shared_ptr<Model>>& getModels() const;

    std::vector<Light> lights;
    Camera camera;
    std::vector<std::shared_ptr<Model>> models;
    
};

#endif // SCENE_H
