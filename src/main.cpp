#include "scene/Scene.h"
#include "core/PathTracer.h"
#include "scene/Model.h"
#include <iostream>
#include <vector>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


int main() {
    Scene scene;

    scene.extractSceneDataFromXML("assets/scene_v3.xml", scene.lights, scene.camera);
    scene.loadModelsFromDirectory("assets/models");
    std::cout <<std::endl<< "Finish loading models." << std::endl;

    

    // ����·��׷����
    PathTracer pathTracer;

    // ��Ⱦ����������ΪͼƬ
    pathTracer.render(scene, scene.camera, 1280, 720, 16);  // ��Ⱦ 800x600 �ֱ��ʵ�ͼ��

    std::cout << "Rendering complete!" << std::endl;
    return 0;
}
