#include "scene/Scene.h"

int main() {
    // ��������
    Scene scene;

    // ����ģ�ͣ�����·�����ж��.obj�ļ���
    scene.loadModelsFromDirectory("assets/models");

    // �������
    Camera camera = { glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f), 45.0f };
    scene.setCamera(camera);

    // ��ӹ�Դ
    Light light = { glm::vec3(10.0f, 10.0f, 10.0f), glm::vec3(1.0f, 1.0f, 1.0f), 1.0f };
    scene.addLight(light);

    // ��Ⱦ����
    scene.render();

    return 0;
}
