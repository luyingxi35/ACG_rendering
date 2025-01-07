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
    std::cout << scene.triangles.size() << std::endl;

    // create pathTracer and BVH Tree
    PathTracer pathTracer;
    BVH bvh(scene);

    // rendering the scene and output the picture
    //pathTracer.render(scene, scene.camera, bvh, 2560, 1440, 64, 16);
    pathTracer.render(scene, scene.camera, bvh, 1280, 640, 128, 16, 0, 1);
	//the last term is IsCartoon, 0 for normal rendering, 1 for cartoon rendering

    std::cout << "Rendering complete!" << std::endl;
    return 0;
}
