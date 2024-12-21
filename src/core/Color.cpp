#include "Color.h"
#include <algorithm>  // For std::min and std::max

// Ĭ�Ϲ��캯������ʼ����ɫΪ��ɫ
Color::Color() : r_(0.0f), g_(0.0f), b_(0.0f) {}

// ���캯������ʼ��Ϊָ���� RGB ֵ
Color::Color(float r, float g, float b) : r_(r), g_(g), b_(b) {}

// ������ɫֵ
void Color::set(float r, float g, float b) {
    r_ = r;
    g_ = g;
    b_ = b;
}

// �ӷ���������أ���������ɫ���
Color Color::operator+(Color other) {
    return Color(r_ + other.r_, g_ + other.g_, b_ + other.b_);
}

// �˷���������أ������˷�
Color Color::operator*(float scalar) {
    return Color(r_ * scalar, g_ * scalar, b_ * scalar);
}

// �˷���������أ�������ɫ�ĵ��
Color Color::operator*(Color other)  {
    return Color(r_ * other.r_, g_ * other.g_, b_ * other.b_);
}

// ��һ����ɫ������ȷ��ÿ����ɫ������ [0, 1] ��Χ��
void Color::clamp() {
    r_ = std::min(1.0f, std::max(0.0f, r_));
    g_ = std::min(1.0f, std::max(0.0f, g_));
    b_ = std::min(1.0f, std::max(0.0f, b_));
}

// ������ɫ�ĻҶ�ֵ
float Color::grayscale()  {
    return 0.2126f * r_ + 0.7152f * g_ + 0.0722f * b_;  // �����ļ�Ȩ�Ҷ�ֵ��ʽ
}
