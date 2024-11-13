#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "scene/Model.h" 

// 渲染窗口
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

    // 设置 OpenGL 状态
    glEnable(GL_DEPTH_TEST);

    // 创建模型对象并加载
    Model model;
    model.loadModel("assets/models/Mesh000.obj");  // 替换为你的 .obj 文件路径

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        renderLoop(model);
    }

    glfwTerminate();
    return 0;
}
