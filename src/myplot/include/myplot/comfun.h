#ifndef COMFUN_H_
#define COMFUN_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <matplotlibcpp.h> //绘图库
#include <random> //随机数
#include <chrono> //系统时钟

namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define m_pi (float)(3.14159)
//使用静态成员函数完成常用函数类库的封装
class COMFUN
{
    public:
        COMFUN();
        ~COMFUN();
        static float ssa(float angle); //角度映射
        static float ssa(float angle, char* unit); //角度映射重载
        static void mymodelplot(Eigen::Matrix<float,2,1> pos,float psi, std::string model_color); //模型可视化
        static void myWAMVplot(Eigen::Matrix<float,2,1> pos, float psi, std::string model_color); //WAMV模型可视化
        static float mywhitenoise(); //白噪声生成


    private:
};

#endif