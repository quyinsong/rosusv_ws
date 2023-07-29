#include "wamv_model.h"

WAMV14::WAMV14(float ts, int integration_step, Eigen::Matrix<float,6,1> x_init)
{
    this->x = x_init;
    this->Thrust<<0,0;
    this->Ths=0.1;
    this->Thrustdot_max = 50;
    this->Thrust_max = 204;
    this->m_sample_time = ts;
    this->m_integration_step = integration_step;
    this->m_dt = m_sample_time/m_integration_step;
    std::cout<<"无人艇初始状态:u:"<<x(0)<<",v:"<<x(1)<<",r:"<<x(2)
        <<",x:"<<x(3)<<",y:"<<x(4)<<",psi:"<<x(5)<<std::endl;
}
WAMV14::WAMV14(Eigen::Matrix<float, 6, 1>  x0,Eigen::Matrix<float, 2, 1>  Thrust0, float ts, int integration_step)
{
    this->x = x0;
    this->Thrust = Thrust0;
    this->Ths=0.5;
    this->Thrustdot_max = 50;
    this->Thrust_max = 204;
    this->m_sample_time = ts;
    this->m_integration_step = integration_step;
    this->m_dt = m_sample_time/m_integration_step;
}

WAMV14::~WAMV14()
{

}

//状态更新
void WAMV14::state_update(Eigen::Matrix<float, 2, 1> Thrust_c,Eigen::Matrix<float,3,1> tau_w)
{
    for(int j=0;j<m_integration_step;j++)
    {
        // 实时获取无人艇状态
        float u = this->x(0);
        float v = this->x(1);
        float r = this->x(2);
        float psi = this->x(5);
        float U = sqrt(pow(u,2)+pow(v,2));
        // 无人艇参数
        // float Xudot = -11.25, Nvdot = -679.8304, Iz = 400, Yvdot = -195.6398, 
        // Nrdot = -600.2102, xg = 0, Yrdot = -54.3864;
        float Xudot = -11.25, Nvdot = 0.2, Iz = 400, Yvdot = -195.6398, 
        Nrdot = -600.2102, xg = 0, Yrdot = 0.2;
        float Xu = -50.5870, Xuu = -5.8722;
        float Yv = -20*rho*abs(v)*2.1092;
        float Nr = -(float)(0.02)*rho*M_PI*U*pow(T,2)*pow(L,2);
        float Nv = -(float)(0.06)*rho*M_PI*U*pow(T,2)*L;
        float Yr = -6*rho*M_PI*U*pow(T,2)*L;
        float Yvv = -599.3130, Yrv = -524.3989, Yvr = -524.3989, Yrr = -1378;
        float Nvv = -524.3989, Nrv = -1378, Nvr = -1378, Nrr = -2996;
        float m11 = m-Xudot;
        float m22 = m-Yvdot;
        float m23 = m*xg-Yrdot;
        float m32 = m*xg-Nvdot;
        float m33 = Iz-Nrdot;
        float c13 = -m*v+(Yvdot*v+(Yrdot+Nvdot)*r/2)/200;
        float c23 = m*u-Xudot*u;
        float c31 = -c13, c32 = -c23;
        float d11 = -Xu-Xuu*abs(u);
        float d22 = -Yv-Yrv*abs(v)-Yvr*abs(r);
        float d23 = -Yr-Yrv*abs(v)-Yrr*abs(r);
        float d32 = -Nv-Nvv*abs(v)-Nvr*abs(r);
        float d33 = -Nr-Nrv*abs(v)-Nrr*abs(r);
        //系统矩阵描述
        Eigen::Matrix<float,3,3> Rbn;
        Rbn << cos(psi), -sin(psi), 0,
            sin(psi), cos(psi), 0,
            0, 0, 1;
        Eigen::Matrix<float,3,3> M;
        M << m11,0,0,
            0,m22,m23,
            0,m32,m33;
        Eigen::Matrix<float,3,3> Crb;
        Crb << 0,0,c13,
            0,0,c23,
            c31,c32,0;
        Eigen::Matrix<float,3,3> Dv;
        Dv << d11,0,0,
            0,d22,d23,
            0,d32,d33;
        //无人艇推力速度
        Eigen::Matrix<float,2,1> Thrustdot;
        Thrustdot = -(Thrust-Thrust_c)/Ths;
        // 无人艇推力速率限幅
        for(int i=0;i<2;i++)
        {
            if(abs(Thrustdot(i))>=Thrustdot_max)
            {
                Thrustdot(i) = Thrustdot_max*mySign(Thrustdot(i));
            }
        }
        //推力更新
        Thrust = Thrustdot*m_dt+Thrust;
        
        // 无人艇推力限幅   
        for(int i=0;i<2;i++)
        {
            if(abs(Thrust(i))>=Thrust_max)
            {
                Thrust(i) = Thrust_max*mySign(Thrust(i));
            }
        }
        //计算总扰动项
        Eigen::Matrix<float,3,1> nu;
        nu << u,v,r;
        Fur = M.inverse()*(-Crb*nu-Dv*nu+tau_w);
        //无人艇状态更新
        Eigen::Matrix<float,3,2> BT;
        BT << 1,1,0,0,B/2,-B/2;
        Eigen::Matrix<float,3,1> tau;
        tau = BT*Thrust;
        Eigen::Matrix<float,3,1> nudot;
        nudot = M.inverse()*tau+Fur;
        Eigen::Matrix<float,3,1> etadot;
        etadot = Rbn*nu;
        Eigen::Matrix<float,6,1> xdot;
        
        xdot<< nudot(0),nudot(1),nudot(2),etadot(0),etadot(1),etadot(2);
        
        x = xdot*m_dt+x;
    }  
}
//获取当前状态
Eigen::Matrix<float,6,1> WAMV14::get_states()
{
    return x;
}
//获取当前推力
Eigen::Matrix<float,2,1> WAMV14::get_thrust()
{
    return Thrust;
}

//获取模型总扰动项
Eigen::Matrix<float,3,1> WAMV14::get_disturbance()
{
    return Fur;
}
//符号函数
int WAMV14::mySign(float x)
{
    if (x<0)
        return -1;
    else if(x=0.0)
        return 0;
    else
        return 1;
}

