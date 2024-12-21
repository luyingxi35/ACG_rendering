#include "Color.h"
#include <algorithm>  // For std::min and std::max

// 默认构造函数，初始化颜色为黑色
Color::Color() : r_(0.0f), g_(0.0f), b_(0.0f) {}

// 构造函数，初始化为指定的 RGB 值
Color::Color(float r, float g, float b) : r_(r), g_(g), b_(b) {}

// 设置颜色值
void Color::set(float r, float g, float b) {
    r_ = r;
    g_ = g;
    b_ = b;
}

// 加法运算符重载：将两种颜色相加
Color Color::operator+(Color other) {
    return Color(r_ + other.r_, g_ + other.g_, b_ + other.b_);
}

// 乘法运算符重载：标量乘法
Color Color::operator*(float scalar) {
    return Color(r_ * scalar, g_ * scalar, b_ * scalar);
}

// 乘法运算符重载：两种颜色的点乘
Color Color::operator*(Color other)  {
    return Color(r_ * other.r_, g_ * other.g_, b_ * other.b_);
}

// 归一化颜色分量：确保每个颜色分量在 [0, 1] 范围内
void Color::clamp() {
    r_ = std::min(1.0f, std::max(0.0f, r_));
    g_ = std::min(1.0f, std::max(0.0f, g_));
    b_ = std::min(1.0f, std::max(0.0f, b_));
}

// 返回颜色的灰度值
float Color::grayscale()  {
    return 0.2126f * r_ + 0.7152f * g_ + 0.0722f * b_;  // 常见的加权灰度值公式
}
