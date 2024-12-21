#ifndef COLOR_H
#define COLOR_H

class Color {
public:
    // ���캯��
    Color();
    Color(float r, float g, float b);

    // ������ɫ����
    void set(float r, float g, float b);

    // ��ɫ����������
    float r() { return r_; }
    float g() { return g_; }
    float b() { return b_; }

    // ���ؼӷ��ͳ˷������
    Color operator+(Color other) ;
    Color operator*(float scalar) ;
    Color operator*( Color other) ;  // ���

    // ��һ����ɫ����
    void clamp();

    // ������ɫ�ĻҶ�ֵ
    float grayscale() ;

    float r_, g_, b_;
};

#endif // COLOR_H

