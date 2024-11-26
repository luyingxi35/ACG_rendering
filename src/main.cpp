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
    std::cout <<std::endl<< "Finish loading models." << std::endl;

    

    // 创建路径追踪器
    PathTracer pathTracer;

    // 渲染场景并保存为图片
    pathTracer.render(scene, scene.camera, 50, 20, 1);  // 渲染 800x600 分辨率的图像

    std::cout << "Rendering complete!" << std::endl;
    return 0;
}
