#include "nmpc_trajectory_tracking.h"

//轨迹跟踪初始化函数
TrajectoryTrackingNMPC::TrajectoryTrackingNMPC(int predict_step, float sample_time, std::vector<Eigen::Matrix<float,8,1>> trajectory_sequence)
{
    m_predict_step = predict_step;
    m_sample_time = sample_time;
    m_nmpc_stage = -1;
    m_trajectory_sequence = trajectory_sequence;

    m_control_command<<0,0;
    m_task = "TrajectoryTracking";

    //最优解猜测值初始化
    for(int j=0;j<2*m_predict_step;j++)
    {
        m_initial_guess.push_back(0);
    }

    //设置求解器
    set_my_nmpc_solver_for_TrajectoryTracking();
}

TrajectoryTrackingNMPC::~TrajectoryTrackingNMPC()
{

}
//创建轨迹跟踪求解器
void TrajectoryTrackingNMPC::set_my_nmpc_solver_for_TrajectoryTracking()
{
    //模型建立
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::SX psi = casadi::SX::sym("psi");
    casadi::SX u = casadi::SX::sym("u");
    casadi::SX v = casadi::SX::sym("v");
    casadi::SX r = casadi::SX::sym("r"); //状态

    casadi::SX Tu = casadi::SX::sym("Tu");
    casadi::SX Tr = casadi::SX::sym("Tr"); //推力

    casadi::SX states = casadi::SX::vertcat({u,v,r,x,y,psi});
    casadi::SX controls = casadi::SX::vertcat({Tu,Tr});
    int n_states = states.size1(); //共有n个状态-----6
    int n_controls = controls.size1(); //共有n个控制输入-----2

    // std::cout<<"n_states: "<< n_states<< endl;
    // std::cout<<"n_controls: "<< n_controls<< endl;

    //运动学模型参数
    casadi::SX Us = casadi::SX::sqrt(u*u+v*v);
    casadi::SX Xudot = -11.25, Nvdot = 0.2, Iz = 400, Yvdot = -195.6398;
    casadi::SX Nrdot = -600.2102, xg = 0, Yrdot = 0.2;
    casadi::SX Xu = -50.5870, Xuu = -5.8722;

    // casadi::SX Yv = -20*rho*abs(v)*2.1092;
    casadi::SX Yv = 100;
    
    casadi::SX Nr = -(float)(0.02)*rho*M_PI*Us*pow(T,2)*pow(L,2);
    casadi::SX Nv = -(float)(0.06)*rho*M_PI*Us*pow(T,2)*L;
    casadi::SX Yr = -6*rho*M_PI*Us*pow(T,2)*L;
    // casadi::SX Nr = -(float)(0.02)*rho*M_PI*1.5*pow(T,2)*pow(L,2);
    // casadi::SX Nv = -(float)(0.06)*rho*M_PI*1.5*pow(T,2)*L;
    // casadi::SX Yr = -6*rho*M_PI*1.5*pow(T,2)*L;

    casadi::SX Yvv = -599.3130, Yrv = -524.3989, Yvr = -524.3989, Yrr = -1378;
    casadi::SX Nvv = -524.3989, Nrv = -1378, Nvr = -1378, Nrr = -2996;
    casadi::SX m11 = m-Xudot;
    casadi::SX m22 = m-Yvdot;
    casadi::SX m23 = m*xg-Yrdot;
    casadi::SX m32 = m*xg-Nvdot;
    casadi::SX m33 = Iz-Nrdot;

    casadi::SX c13 = -m*v+(Yvdot*v+(Yrdot+Nvdot)*r/2)/200;
    casadi::SX c23 = m*u-Xudot*u;
    // casadi::SX c13 = 0;
    // casadi::SX c23 = 0;

    casadi::SX c31 = -c13, c32 = -c23;

    // casadi::SX d11 = -Xu-Xuu*casadi::SX::abs(u);
    // casadi::SX d22 = -Yv-Yrv*casadi::SX::abs(v)-Yvr*casadi::SX::abs(r);
    // casadi::SX d23 = -Yr-Yrv*casadi::SX::abs(v)-Yrr*casadi::SX::abs(r);
    // casadi::SX d32 = -Nv-Nvv*casadi::SX::abs(v)-Nvr*casadi::SX::abs(r);
    // casadi::SX d33 = -Nr-Nrv*casadi::SX::abs(v)-Nrr*casadi::SX::abs(r);
    // casadi::SX d11 = -Xu;
    // casadi::SX d22 = -Yv;
    // casadi::SX d23 = 0;
    // casadi::SX d32 = 0;
    // casadi::SX d33 = -Nr;

    //运动学模型
    // casadi::SX rhs = casadi::SX::vertcat({
    //     (-c13*r-d11*u+Tu)/m11,
    //     (-c23*r-d22*v-d23*r)/m22,
    //     (-c31*u-c32*v-d32*v-d33*r+Tr)/m33,
    //     u*casadi::SX::cos(psi)-v*casadi::SX::sin(psi),
    //     u*casadi::SX::sin(psi)+v*casadi::SX::cos(psi),
    //     r
    // });
    // casadi::SX rhs = casadi::SX::vertcat({
    //     (-50*u+Tu-c13*r)/m11,
    //     (-150*v-c23*r)/m22,
    //     (-15*r+Tr-c31*u-c32*v)/m33,
    //     u*casadi::SX::cos(psi)-v*casadi::SX::sin(psi),
    //     u*casadi::SX::sin(psi)+v*casadi::SX::cos(psi),
    //     r
    // });   
    casadi::SX rhs = casadi::SX::vertcat({
        (-50*u+Tu)/m11,
        (-150*v)/m22,
        (-15*r+Tr)/m33,
        u*casadi::SX::cos(psi)-v*casadi::SX::sin(psi),
        u*casadi::SX::sin(psi)+v*casadi::SX::cos(psi),
        r
    });   

    //定义模型函数
    casadi::Function m_f = casadi::Function("f", {states, controls}, {rhs});

    // 求解问题符号表示
    casadi::SX U = casadi::SX::sym("U",n_controls,m_predict_step); //待求解的控制变量
    casadi::SX X = casadi::SX::sym("X",n_states,m_predict_step+1); //系统状态

    //优化参数（需要给出当前运动状态和预测视野内的期望点或者期望轨迹）
    casadi::SX opt_para = casadi::SX::sym("opt_para",n_states+(n_states+n_controls)*m_predict_step);
    //优化变量（需要求解的控制序列）
    casadi::SX opt_var = casadi::SX::reshape(U.T(),-1,1);

    //根据上述模型函数向前预测无人艇运动状态
    X(casadi::Slice(),0) = opt_para(casadi::Slice(0,6,1)); //状态初始值

    for(int i=0;i<m_predict_step;i++)
    {
        std::vector<casadi::SX> input_X;
        casadi::SX X_current = X(casadi::Slice(),i);
        casadi::SX U_current = U(casadi::Slice(),i);
        input_X.push_back(X_current);
        input_X.push_back(U_current);
        X(casadi::Slice(),i+1) = m_f(input_X).at(0)*m_sample_time+X_current;
    }

    //控制序列与输出的关系函数（预测函数）
    m_predict_fun = casadi::Function("m_predict_fun",{U,opt_para},{X});

    //惩罚矩阵
    casadi::SX m_Q = casadi::SX::zeros(6,6);
    casadi::SX m_R = casadi::SX::zeros(2,2);
    m_Q(0,0) = 0.01;
    m_Q(1,1) = 0.01;
    m_Q(2,2) = 0.01;
    m_Q(3,3) = 60;
    m_Q(4,4) = 60;
    m_Q(5,5) = 200;
    m_R(0,0) = 0.005;
    m_R(1,1) = 0.005;
    
    //计算代价函数
    casadi::SX cost_fun = casadi::SX::sym("cost_fun");
    cost_fun = 0;

    casadi::SX Xref = casadi::SX::reshape(opt_para(casadi::Slice(6,m_predict_step*8+6)),8,m_predict_step);
    for(int k=0;k<m_predict_step;k++)
    {
        casadi::SX states_err = X(casadi::Slice(),k)-Xref(casadi::Slice(0,6,1),k);
        // casadi::SX controls_err = U(casadi::Slice(),k)-Xref(casadi::Slice(6,8,1),k);
        casadi::SX controls_err = U(casadi::Slice(),k);
        cost_fun = cost_fun+casadi::SX::mtimes({states_err.T(),m_Q,states_err})+
                            casadi::SX::mtimes({controls_err.T(),m_R,controls_err});
    }

    //构建求解器(暂时不考虑约束)
    casadi::SXDict nlp_prob= {
        {"f", cost_fun},
        {"x", opt_var},
        {"p",opt_para}
    };

    std::string solver_name = "ipopt";
    casadi::Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 5000;
    nlp_opts["ipopt.print_level"] = 0;
    nlp_opts["print_time"] = 0;
    nlp_opts["ipopt.acceptable_tol"] =  1e-6;
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

    m_solver = nlpsol("nlpsol", solver_name, nlp_prob, nlp_opts);

}

//轨迹跟踪优化求解函数
void TrajectoryTrackingNMPC::opti_solution_for_TrajectoryTracking(Eigen::Matrix<float,6,1> current_states)
{
    //设置控制约束
    std::vector<float> lbx;
    std::vector<float> ubx;
    std::vector<float> parameters;
    for (int j = 0; j < m_predict_step; j++)
    {
        lbx.push_back(-300);

        ubx.push_back(300);
    }
    for (int j = 0; j < m_predict_step; j++)
    {
        lbx.push_back(-100);

        ubx.push_back(100);
    }
    //设置求解器输入参数
    m_nmpc_stage++;
    for(int j=0;j<6;j++)
    {
        parameters.push_back(current_states[j]);
    }
    int index_end;
    index_end = m_nmpc_stage+m_predict_step;
    if(index_end<m_trajectory_sequence.size()-1)
    {
        for(int i=m_nmpc_stage;i<index_end;i++)
        {
            for(int j=0;j<8;j++)
            {
                parameters.push_back(m_trajectory_sequence.at(i)[j]);
            }
        }
    }
    else
    {
        if(m_nmpc_stage<m_trajectory_sequence.size()-1)
        {
            int k_index=0; //记录已经填充几段轨迹
            for(int i=m_nmpc_stage;i<m_trajectory_sequence.size()-1;i++)
            {
                k_index++;
                for(int j=0;j<8;j++)
                {
                    parameters.push_back(m_trajectory_sequence.at(i)[j]);
                }
            }
            //填充剩余的轨迹数据
            for(int i=0;i<m_predict_step-k_index;i++)
            {
                for(int j=0;j<8;j++)
                {
                    parameters.push_back(m_trajectory_sequence.at(m_trajectory_sequence.size()-1)[j]);
                }
            }     
        }
        else
        {
            for(int i=0;i<m_predict_step;i++)
            {
                for(int j=0;j<8;j++)
                {
                    parameters.push_back(m_trajectory_sequence.at(m_trajectory_sequence.size()-1)[j]);
                }
            }    
        }
    
    }

    //求解参数设置
    m_args["lbx"] = lbx;
    m_args["ubx"] = ubx;
    m_args["x0"] = m_initial_guess;
    m_args["p"] = parameters;
    //求解
    m_res = m_solver(m_args);

    //获取代价函数
    casadi::SX cost_f = m_res.at("f");
    std::cout<<"代价值: "<<cost_f<<std::endl;

    //获取优化变量
    std::vector<float> res_control_all(m_res.at("x"));

    std::vector<float> res_control_Tu, res_control_Tr;

    res_control_Tu.assign(res_control_all.begin(), res_control_all.begin() + m_predict_step);
    res_control_Tr.assign(res_control_all.begin() + m_predict_step, res_control_all.begin() + 2 * m_predict_step);

    //存储当前控制序列作为下一时刻的最优解猜测值
    std::vector<float> initial_guess;
    for (int j = 0; j < m_predict_step-1; j++)
    {
        initial_guess.push_back(res_control_Tu.at(j+1));
    }
    initial_guess.push_back(res_control_Tu.at(m_predict_step-1));
    for (int j = 0; j < m_predict_step-1; j++)
    {
        initial_guess.push_back(res_control_Tr.at(j+1));
    }
    initial_guess.push_back(res_control_Tr.at(m_predict_step-1));
    m_initial_guess = initial_guess;

    // 采用求解得到的控制序列的第一组作为当前控制量
    m_control_command << res_control_Tu.front(), res_control_Tr.front();

    //预测轨迹
    std::vector<Eigen::Matrix<float,6,1>> predict_trajectory;
    predict_trajectory.push_back(current_states);
    Eigen::Matrix<float,6,1> next_states;
    for(int j=0;j<m_predict_step;j++)
    {
        float u,v,r,x,y,psi;
        u = predict_trajectory.at(j)[0];
        v = predict_trajectory.at(j)[1];
        r = predict_trajectory.at(j)[2];
        x = predict_trajectory.at(j)[3];
        y = predict_trajectory.at(j)[4];
        psi = predict_trajectory.at(j)[5];
        float Xudot = -11.25, Nvdot = 0.2, Yvdot = -195.6398, Yrdot = 0.2;
        float c13 = -m*v+(Yvdot*v+(Yrdot+Nvdot)*r/2)/200;
        float c23 = m*u-Xudot*u;
        float c31 = -c13, c32 = -c23;

        // next_states<<   (-50*u+res_control_Tu.at(j)-c13*r)/161.25*m_sample_time+u,
        //                 (-150*v-c23*r)/345.6398*m_sample_time+v,
        //                 (-15*r+res_control_Tr.at(j)-c31*u-c32*v)/1000.2102*m_sample_time+r,
        //                 (u*std::cos(psi)-v*std::sin(psi))*m_sample_time+x,
        //                 (u*std::sin(psi)+v*std::cos(psi))*m_sample_time+y,
        //                 r*m_sample_time+r;
        next_states<<   (-50*u+res_control_Tu.at(j))/161.25*m_sample_time+u,
                        (-150*v)/345.6398*m_sample_time+v,
                        (-15*r+res_control_Tr.at(j))/1000.2102*m_sample_time+r,
                        (u*std::cos(psi)-v*std::sin(psi))*m_sample_time+x,
                        (u*std::sin(psi)+v*std::cos(psi))*m_sample_time+y,
                        r*m_sample_time+r;

        predict_trajectory.push_back(next_states);
    }


    m_predict_trajectory = predict_trajectory;

}

Eigen::Matrix<float,2,1> TrajectoryTrackingNMPC::get_controls()        // casadi::SX c13 = 0;
        // casadi::SX c23 = 0;
{

    return m_control_command;
}

std::vector<Eigen::Matrix<float, 6, 1>> TrajectoryTrackingNMPC::get_predict_trajectory()
{
    return m_predict_trajectory;
}

