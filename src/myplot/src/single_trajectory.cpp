#include "single_trajectory.h"


SingleTTG::SingleTTG(float ts, float total_time, int integration_step,  Eigen::Matrix<float,6,1> x_init)
{
    m_sample_time = ts;
    m_total_time = total_time;
    m_total_step = m_total_time/m_sample_time;
    m_x_init = x_init;
    m_integration_step = integration_step;
    m_wamv = new WAMV14(m_sample_time, integration_step, m_x_init);
}
SingleTTG::~SingleTTG()
{
    delete m_wamv;
}

std::vector<Eigen::Matrix<float,8,1>> SingleTTG::TTG()
{
    Eigen::Matrix<float,3,1> tau_w;
    tau_w << 0, 0, 0;
    std::vector<Eigen::Matrix<float,8,1>> trajectory_points;
    for(int k=1; k<=m_total_step; k++)
    {
        float t;
        t = (k-1)*m_sample_time;
        Eigen::Matrix<float,2,1> Thrust_c;
        Thrust_c<<100,100;
        if(t>=20)
        {
            Thrust_c << 120,100;
        }

        if(t>=40)
        {
            Thrust_c << 100,120;
        }

        if(t>=80)
        {
            Thrust_c << 120,100;
        }

        if(t>=120)
        {
            Thrust_c << 0,0;
        }
        Eigen::Matrix<float,6,1> x;
        x = m_wamv->get_states();
        m_wamv->state_update(Thrust_c,tau_w);

        Eigen::Vector2f tau;
        Eigen::Matrix<float,2,2> BT;
        BT << 1,1,1.83/2,-1.83/2;
        tau = BT*Thrust_c;

        Eigen::Matrix<float,8,1> outputs;
        outputs<<x,tau;
        trajectory_points.push_back(outputs);

        // std::cout<< outputs<<endl;
        
    }
    return trajectory_points;
}


        
