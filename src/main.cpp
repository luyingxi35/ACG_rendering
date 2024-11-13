#include "scene/Scene.h"

int main() {
    // 创建场景
    Scene scene;

    // 加载模型（假设路径中有多个.obj文件）
    scene.loadModelsFromDirectory("assets/models");

    // 设置相机
    Camera camera = { glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 45.0f };
    scene.setCamera(camera);

    // 添加光源
    Light light = { glm::vec3(10.0f, 10.0f, 10.0f), glm::vec3(1.0f, 1.0f, 1.0f), 1.0f };
    scene.addLight(light);

    // 渲染场景
    scene.render();

    return 0;
}
