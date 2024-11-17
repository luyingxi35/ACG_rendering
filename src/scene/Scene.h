#ifndef SCENE_H
#define SCENE_H

#include "Model.h"
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <iostream>
#include <filesystem>
#include <memory>
#include <pugixml.hpp>

// ��Դ��
struct Light {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;
};

// �����
struct Camera {
    glm::vec3 position;
    glm::vec3 direction;
    glm::vec3 up;
    float fov;
};

// ������
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
    const std::vector<Light>& getLights() const;

    void loadModelsFromDirectory(const std::string& directory);

    void render(); // ģ����Ⱦ���̣�����ӡ��Ϣ��
    std::vector<Light> lights;
    Camera camera;

private:
    std::vector<std::shared_ptr<Model>> models;
    
};

#endif // SCENE_H
