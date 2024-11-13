#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "scene/Model.h" 

// ��Ⱦ����
void renderLoop(Model& model) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    model.draw();
    glfwSwapBuffers(glfwGetCurrentContext());
}

int main() {
    if (!glfwInit()) {
        std::cerr << "GLFW initialization failed!" << std::endl;
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 600, "Model Loader", nullptr, nullptr);
    if (!window) {
        std::cerr << "GLFW window creation failed!" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glewInit();

    // ���� OpenGL ״̬
    glEnable(GL_DEPTH_TEST);

    // ����ģ�Ͷ��󲢼���
    Model model;
    model.loadModel("assets/models/Mesh000.obj");  // �滻Ϊ��� .obj �ļ�·��

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        renderLoop(model);
    }

    glfwTerminate();
    return 0;
}
