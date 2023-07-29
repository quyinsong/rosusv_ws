#ifndef WAMV14_H_
#define WAMV14_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

class WAMV14
{
    public:
        //构造函数
        WAMV14(float ts, int integration_step, Eigen::Matrix<float,6,1> x_init);
        WAMV14(Eigen::Matrix<float, 6, 1>  x0,Eigen::Matrix<float, 2, 1>  Thrust0, float ts, int integration_step);
        //析构函数
        ~WAMV14();
        //状态更新
        void state_update(Eigen::Matrix<float, 2, 1> Thrust_c,Eigen::Matrix<float,3,1> tau_w);
        //获取当前状态
        Eigen::Matrix<float,6,1> get_states();
        //获取当前推力
        Eigen::Matrix<float,2,1> get_thrust();
        //获取当前模型总扰动项
        Eigen::Matrix<float,3,1> get_disturbance();
        //符号函数
        int mySign(float x);

    private:
        Eigen::Matrix<float,6,1> x; //无人艇状态
        Eigen::Matrix<float,2,1> Thrust; //无人艇推力状态
        float Thrustdot_max; //推力速度限幅
        float Thrust_max; //推力限幅
        float Ths; //无人艇推力积分常数
        float L = 4.29;  // length
        float T = 0.127; // draft
        float Bhull = 0.37; // 浮筒beam
        float B = 1.83; // 浮筒中心间距
        float rho = pow(10,3); // 水的密度
        float m = 150; // mass
        float Cd = 1.1; // 方形系数
        float BOA = 2.2; //width
        Eigen::Matrix<float,3,1> Fur; //总扰动 

        float m_sample_time; //采样步长
        int m_integration_step; //积分步长
        float m_dt; //单次积分时间
};
#endif
