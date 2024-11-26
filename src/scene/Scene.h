#ifndef SCENE_H
#define SCENE_H

#include <pugixml.hpp>
#include "Model.h"



class Scene {
public:
    Scene();
    ~Scene();

    void addModel(std::shared_ptr<Model> model);
    Material extractMaterialFromBSDF(pugi::xml_node bsdf);
    void extractSceneDataFromXML(const std::string& xmlPath, std::vector<Light>& lights, Camera& camera);

    std::vector<Light> lights;
    Camera camera;
    std::vector<std::shared_ptr<Model>> models;
    
};

#endif // SCENE_H
