#include "Scene.h"

namespace fs = std::filesystem;

Scene::Scene() {
    camera = { glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 45.0f };
}

Scene::~Scene() {}

void Scene::addModel(std::shared_ptr<Model> model) {
    models.push_back(model);
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
    pugi::xml_node cameraNode = doc.child("scene").child("camera");
    camera.position = glm::vec3(
        cameraNode.child("position").attribute("x").as_float(),
        cameraNode.child("position").attribute("y").as_float(),
        cameraNode.child("position").attribute("z").as_float()
    );
    camera.direction = glm::vec3(
        cameraNode.child("direction").attribute("x").as_float(),
        cameraNode.child("direction").attribute("y").as_float(),
        cameraNode.child("direction").attribute("z").as_float()
    );
    camera.up = glm::vec3(
        cameraNode.child("up").attribute("x").as_float(),
        cameraNode.child("up").attribute("y").as_float(),
        cameraNode.child("up").attribute("z").as_float()
    );
    camera.fov = cameraNode.child("fov").text().as_float();

    std::cout << "Camera loaded: Position(" << camera.position.x << ", " << camera.position.y << ", " << camera.position.z << ")\n";

}

const std::vector<std::shared_ptr<Model>>& Scene::getModels() const {
    return models;
}

const std::vector<Light>& Scene::getLights() const {
    return lights;
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