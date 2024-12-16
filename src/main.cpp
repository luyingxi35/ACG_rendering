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
<<<<<<< HEAD
    pathTracer.render(scene, scene.camera, bvh, 320, 180, 64, 16);  
=======
    pathTracer.render(scene, scene.camera, bvh, 320, 160, 8, 16);  
>>>>>>> ef25931148c4059803385d28f1b881af999e96b6

    std::cout << "Rendering complete!" << std::endl;
    return 0;
}
