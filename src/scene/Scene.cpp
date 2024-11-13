#include "Scene.h"
#include <iostream>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

Scene::Scene() {
    camera = { glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 45.0f };
}

Scene::~Scene() {}

void Scene::addModel(std::shared_ptr<Model> model) {
    models.push_back(model);
}

void Scene::addLight(const Light& light) {
    lights.push_back(light);
}

void Scene::setCamera(const Camera& cam) {
    camera = cam;
}

void Scene::loadModelsFromDirectory(const std::string& directory) {
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".obj") {
            auto model = std::make_shared<Model>();
            model->loadModelFromFile(entry.path().string());
            addModel(model);
        }
    }
}

void Scene::render() {
    std::cout << "Rendering scene with " << models.size() << " models and " << lights.size() << " lights." << std::endl;
    for (const auto& model : models) {
        model->draw();
    }
    for (const auto& light : lights) {
        std::cout << "Light at position: ("
            << light.position.x << ", "
            << light.position.y << ", "
            << light.position.z << ") with color: ("
            << light.color.x << ", "
            << light.color.y << ", "
            << light.color.z << ") and intensity: "
            << light.intensity << std::endl;
    }
    std::cout << "Camera position: ("
        << camera.position.x << ", "
        << camera.position.y << ", "
        << camera.position.z << ") looking towards: ("
        << camera.direction.x << ", "
        << camera.direction.y << ", "
        << camera.direction.z << ") with FOV: "
        << camera.fov << std::endl;
}