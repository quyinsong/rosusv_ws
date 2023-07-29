#include <ros/ros.h>
#include "wamv_model/states.h"
#include "comfun.h"
#include "single_trajectory.h"


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//由于回调函数不能返回数值，若想处理回调函数接收的话题消息，需要定义如下Listener类，
//利用类中成员来处理接收到的消息
class Listener
{
public:
    std::vector<float> x_data;
    std::vector<float> y_data;
    Eigen::Matrix<float,2,1> pos;
    float psi;
    std::string model_color;
    std::string model_color1;

public:
    Listener();
    ~Listener();
    void callback(const wamv_model::states::ConstPtr& msg);
};

Listener::Listener()
{
    model_color = "red";
    model_color1 = "blue";
}

Listener::~Listener()
{

}

void Listener::callback(const wamv_model::states::ConstPtr& msg)
{
    // ROS_INFO("I heard states: [%f, %f, %f]", msg->x, msg->y, msg->theta);
    x_data.push_back(msg->x);
    y_data.push_back(msg->y);

    pos<<msg->x,msg->y;
    psi = msg->psi;

    // ROS_INFO("current states: [%f, %f, %f]", pos[0], pos[1], psi);
}

//主函数
int main(int argc, char **argv)
{
    //初始化ros节点
    ros::init(argc, argv, "test_plot_ttc");

    ros::NodeHandle n;

    //创建无人艇状态监听对象
    Listener mlistener;

    ros::Subscriber sub = n.subscribe("/wamv_states",10,&Listener::callback,&mlistener);

    //创建无人艇期望位置监听对象
    Listener mlistener1;

    ros::Subscriber sub1 = n.subscribe("/desired_pos",10,&Listener::callback,&mlistener1);

    //生成轨迹
    float sample_time = 0.02;
    float total_time = 200;
    int integration_step=1;
    Eigen::Matrix<float,6,1> x_init;
    x_init<<0,0,0,5,5,M_PI/2;
    SingleTTG ttg(sample_time,total_time,integration_step,x_init);
    std::vector<Eigen::Matrix<float,8,1>> trajectory_sequence;
    trajectory_sequence = ttg.TTG();

    std::vector<float> trajectory_points_x;
    std::vector<float> trajectory_points_y;
    std::vector<float> trajectory_points_psi;  //用于绘图
    for(int k=0;k<trajectory_sequence.size();k++)
    {
      trajectory_points_x.push_back(trajectory_sequence.at(k)(3));
      trajectory_points_y.push_back(trajectory_sequence.at(k)(4));
      trajectory_points_psi.push_back(trajectory_sequence.at(k)(5));
    }

    Eigen::Matrix<float,2,1> trajectory_pos; //轨迹点位置向量
    float trajectory_psi; //轨迹点艏向角

    // 设置循环的频率
    ros::Rate loop_rate(50);

    int k=0;
    
    while(ros::ok())
    {
        ros::spinOnce();
        //该节点执行次数
        k=k+1;
        // std::cout<<"k: "<<k<<std::endl;
        //绘图
        plt::figure(1);
        if(k%10==0)
        {
            plt::clf();
            plt::cla();
            //绘制船舶模型
            COMFUN::myWAMVplot(mlistener.pos,mlistener.psi,mlistener.model_color);
            plt::plot(mlistener.y_data,mlistener.x_data,mlistener.model_color);

            //绘制全局期望轨迹
            plt::plot(trajectory_points_y,trajectory_points_x,{{"color", "blue"}, {"linestyle", "-"}});

            //绘制当前期望位置点
            // if(k<=trajectory_sequence.size())
            // {
            //     trajectory_pos <<trajectory_points_x.at(k-1), trajectory_points_y.at(k-1);
            //     trajectory_psi = trajectory_points_psi.at(k-1);
            // }else{
            //     trajectory_pos<<trajectory_points_x.at(trajectory_points_x.size()-1),
            //                     trajectory_points_y.at(trajectory_points_y.size()-1);
            //     trajectory_psi = trajectory_points_psi.at(trajectory_points_psi.size()-1);

            // }
            // COMFUN::myWAMVplot(trajectory_pos,trajectory_psi,mlistener.model_color1);
            COMFUN::myWAMVplot(mlistener1.pos,mlistener1.psi,mlistener1.model_color1);

            plt::pause(0.1);
        }

        // ROS_INFO("current states: [%f, %f, %f]", mlistener.pos[0], mlistener.pos[1], mlistener.psi);
        loop_rate.sleep();
    }
 

  return 0;
}