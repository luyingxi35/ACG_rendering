#ifndef COLOR_H
#define COLOR_H

class Color {
public:
    // 构造函数
    Color();
    Color(float r, float g, float b);

    // 设置颜色分量
    void set(float r, float g, float b);

    // 颜色分量访问器
    float r() { return r_; }
    float g() { return g_; }
    float b() { return b_; }

    // 重载加法和乘法运算符
    Color operator+(Color other) ;
    Color operator*(float scalar) ;
    Color operator*( Color other) ;  // 点乘

    // 归一化颜色分量
    void clamp();

    // 返回颜色的灰度值
    float grayscale() ;

    float r_, g_, b_;
};

#endif // COLOR_H

