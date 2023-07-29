#include <ros/ros.h>
#include "wamv_model/states.h"
#include "wamv_model/controls.h"
#include "nmpc_trajectory_tracking.h"
#include "single_trajectory.h"

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
    ros::init(argc, argv, "test_trajectory_tracking");

    ros::NodeHandle n;

    //生成轨迹
    float sample_time = 0.02;
    float total_time = 200;
    int integration_step=1;
    Eigen::Matrix<float,6,1> x_init;
    x_init<<0,0,0,5,5,M_PI/2; 
    SingleTTG singlettg(sample_time,total_time,integration_step,x_init);
    std::vector<Eigen::Matrix<float,8,1>> trajectory_sequence;
    trajectory_sequence = singlettg.TTG();

    //创建NMPC控制器
    int predict_step = 50;

    TrajectoryTrackingNMPC nmpc(predict_step, sample_time, trajectory_sequence);

    //创建监听对象
    Listener mlistener;

    ros::Subscriber sub = n.subscribe("/wamv_states",10,&Listener::callback,&mlistener);

    //创建发布对象
    ros::Publisher pub_controls =n.advertise<wamv_model::controls>("/nmpc_controls",10);
    ros::Publisher pub_desired_pos =n.advertise<wamv_model::states>("/desired_pos",10);

    // 设置循环的频率
    ros::Rate loop_rate(50);

    int count=0;
    while(ros::ok())
    {
        //循环回调函数
        ros::spinOnce();
        
        //该节点执行次数
        count++;
        // std::cout<<"count: "<<count<<std::endl;

        //调用nmpc求解
        nmpc.opti_solution_for_TrajectoryTracking(mlistener.current_states);
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
        pub_controls.publish(pub_msg);

        //发布当前期望位置
        Eigen::Vector3f desired_pos;
        desired_pos = nmpc.get_current_desired_pos();
        wamv_model::states pub_msg1;
        pub_msg1.x = desired_pos[0];
        pub_msg1.y = desired_pos[1];
        pub_msg1.psi = desired_pos[2];

        pub_desired_pos.publish(pub_msg1);

        //休眠
        loop_rate.sleep();
    }
 

  return 0;
}