#ifndef SCENE_H
#define SCENE_H

#include "Model.h"
#include <vector>
#include <memory>
#include <string>
#include <glm/glm.hpp>

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

class Scene {
public:
    Scene();
    ~Scene();

    void addModel(std::shared_ptr<Model> model);
    void addLight(const Light& light);
    void setCamera(const Camera& camera);

    void loadModelsFromDirectory(const std::string& directory);

    void render(); // ģ����Ⱦ���̣�����ӡ��Ϣ��

private:
    std::vector<std::shared_ptr<Model>> models;
    std::vector<Light> lights;
    Camera camera;
};

#endif // SCENE_H
