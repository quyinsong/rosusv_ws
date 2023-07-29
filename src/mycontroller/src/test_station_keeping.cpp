#include <ros/ros.h>
#include "wamv_model/states.h"
#include "wamv_model/controls.h"
#include "nmpc_station_keeping.h"


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//由于回调函数不能返回数值，若想处理回调函数接收的话题消息，需要定义如下Listener类，
//利用类中成员来处理接收到的消息
class Listener
{
public:
    Eigen::Matrix<float,6,1> current_states;

public:
    Listener();
    ~Listener();
    void callback(const wamv_model::states::ConstPtr& msg);
};

Listener::Listener()
{
    current_states<<0,0,0,0,0,0;

}

Listener::~Listener()
{

}

void Listener::callback(const wamv_model::states::ConstPtr& msg)
{
    // ROS_INFO("I heard states: [%f, %f, %f]", msg->x, msg->y, msg->theta);

    current_states<<msg->u,msg->v,msg->r,msg->x,msg->y,msg->psi;

    // ROS_INFO("current states: [%f, %f, %f]", pos[0], pos[1], psi);
}

//主函数
int main(int argc, char **argv)
{
    //初始化ros节点
    ros::init(argc, argv, "test_station_keeping");

    ros::NodeHandle n;

    //创建NMPC控制器
    int predict_step = 50;
    float sample_time = 0.1;

    StationKeepingNMPC nmpc(predict_step, sample_time);

    //设定期望最终位置与控制量
    Eigen::Matrix<float,6,1> desired_states;
    desired_states<< 0,0,0,20,20,M_PI/2;

    //创建监听对象
    Listener mlistener;

    ros::Subscriber sub = n.subscribe("/wamv_states",100,&Listener::callback,&mlistener);

    //创建发布对象
    ros::Publisher pub =n.advertise<wamv_model::controls>("/nmpc_controls",100);

    // 设置循环的频率
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        //循环回调函数
        ros::spinOnce();
        
        //调用nmpc求解
        nmpc.opti_solution_for_StationKeeping(mlistener.current_states,desired_states);
        //获取控制量
        Eigen::Vector2f nmpc_controls = nmpc.get_controls();

        //nmpc求解得到的控制量需要进行坐标转换得到最终的期望双桨推力
        Eigen::Matrix<float,2,2> BT;
        BT << 1,1,1.83/2,-1.83/2;
        Eigen::Vector2f Thrust_command;
        Thrust_command = BT.inverse()*nmpc_controls;

        //发布求解得到的控制量
        wamv_model::controls pub_msg;
        pub_msg.thrust_left = Thrust_command[0];
        pub_msg.thrust_right = Thrust_command[1];

        pub.publish(pub_msg);

        //休眠
        loop_rate.sleep();
    }
 

  return 0;
}