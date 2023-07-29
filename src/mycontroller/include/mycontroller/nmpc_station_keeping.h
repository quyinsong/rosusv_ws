#ifndef NMPC_STATION_KEEPING_H_
#define NMPC_STATION_KEEPING_H_

#include <casadi/casadi.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>

class StationKeepingNMPC
{
private:
    /* data */
    // MPC参数
    int m_predict_step; //一次采样间隔预测的步数
    float m_sample_time; //采样时间
    int m_nmpc_stage; //存储第几次调用nmpc,轨迹跟踪获取当前跟踪轨迹需要使用
    std::vector<Eigen::Matrix<float,6,1>> m_predict_trajectory; //预测轨迹
    std::vector<float> m_initial_guess; //最优解猜测值
    std::string m_task; //当前任务设置
    //定点控制中期望点
    Eigen::Matrix<float,6,1> m_desired_states;
    //轨迹序列点
    std::vector<Eigen::Matrix<float,8,1>> m_trajectory_sequence;
    //符号定义
    casadi::Function m_solver; //求解器
    casadi::Function m_predict_fun; //预测函数
    std::map<std::string, casadi::DM> m_res; //求解结果
    std::map<std::string, casadi::DM> m_args; //求解参数

    //求解得到的控制量
    Eigen::Matrix<float, 2, 1> m_control_command; 

    
    //模型参数
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
 
public:
    StationKeepingNMPC(int predict_step, float sample_time);
    StationKeepingNMPC(int predict_step, float sample_time, std::vector<Eigen::Matrix<float,8,1>> trajectory_sequence);
    ~StationKeepingNMPC();
    //定义求解器
    void set_my_nmpc_solver_for_StationKeeping();
    void set_my_nmpc_solver_for_TrajectoryTracking();
    //优化求解
    void opti_solution_for_StationKeeping(Eigen::Matrix<float,6,1> current_states,Eigen::Matrix<float,6,1> desired_states);
    void opti_solution_for_TrajectoryTracking(Eigen::Matrix<float,6,1> current_states);
    //获取最优控制向量
    Eigen::Matrix<float,2,1> get_controls();

    //获得最优控制序列
    std::vector<Eigen::Matrix<float,6,1>> get_predict_trajectory();

};



#endif