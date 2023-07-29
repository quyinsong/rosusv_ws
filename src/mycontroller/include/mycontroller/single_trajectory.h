#ifndef SINGLE_TRAJECTORY_H_
#define SINGLE_TRAJECTORY_H_

#include "wamv_model.h"
#include <iostream>
#include <vector>

class SingleTTG
{
private:
    float m_sample_time; 
    float m_total_time;
    int m_total_step;//仿真总步数
    int m_integration_step;

    Eigen::Matrix<float,6,1> m_x_init;

    WAMV14 *m_wamv;

public:
    SingleTTG(float ts, float total_time, int integration_step, Eigen::Matrix<float,6,1> x_init );
    ~SingleTTG();
    std::vector<Eigen::Matrix<float,8,1>> TTG();


};





#endif

