#include <ros/ros.h>
#include "wamv_model/states.h"
#include "comfun.h"


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

public:
    Listener();
    ~Listener();
    void callback(const wamv_model::states::ConstPtr& msg);
};

Listener::Listener()
{
    model_color = "red";
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
    ros::init(argc, argv, "test_plot");

    ros::NodeHandle n;

    //创建监听对象
    Listener mlistener;

    ros::Subscriber sub = n.subscribe("/wamv_states",100,&Listener::callback,&mlistener);

    // 设置循环的频率
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        //绘图
        plt::figure(1);
        plt::clf();
        plt::cla();
        COMFUN::myWAMVplot(mlistener.pos,mlistener.psi,mlistener.model_color);
        plt::plot(mlistener.y_data,mlistener.x_data,mlistener.model_color);
        plt::pause(0.1);

        ROS_INFO("current states: [%f, %f, %f]", mlistener.pos[0], mlistener.pos[1], mlistener.psi);
        loop_rate.sleep();
    }
 

  return 0;
}