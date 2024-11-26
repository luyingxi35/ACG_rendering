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

    // create pathTracer and BVH Tree
    PathTracer pathTracer;
    BVH bvh(scene);

    // äÖÈ¾³¡¾°²¢±£´æÎªÍ¼Æ¬
    pathTracer.render(scene, scene.camera, bvh, 300, 300, 16);  // äÖÈ¾ 800x600 ·Ö±æÂÊµÄÍ¼Ïñ

    std::cout << "Rendering complete!" << std::endl;
    return 0;
}
