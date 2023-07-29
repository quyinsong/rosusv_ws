#include "comfun.h"

COMFUN::COMFUN()
{

}

COMFUN::~COMFUN()
{

}

// 映射角度关系(利用函数重载)
float COMFUN::ssa(float angle)
{
    float T;
    T = fmod(angle+m_pi,2*m_pi)-m_pi;
    return T;
}
float COMFUN::ssa(float angle, char* unit)
{
    float T;
    angle = fmod(angle+180,(float)360)-180;
    return T;
}

//绘制无人艇模型
void COMFUN::mymodelplot(Eigen::Matrix<float,2,1> pos,float psi, std::string model_color)
{
    //无人艇参数
    float L = 5;  // length
    float BOA = 2; //width
    //船体坐标系下无人艇模型边界点
    Eigen::Matrix<float,2,1> xb1;
    Eigen::Matrix<float,2,1> xb2;
    Eigen::Matrix<float,2,1> xb3;
    Eigen::Matrix<float,2,1> xb4;
    Eigen::Matrix<float,2,1> xb5;
    float L1 = L/4;
    xb1 << L/2-L1,-BOA/2;
    xb2 << L/2,0;
    xb3 << L/2-L1,BOA/2;
    xb4 << -L/2,BOA/2;
    xb5 << -L/2,-BOA/2;
    //旋转矩阵
    Eigen::Matrix<float,2,2> Rbn;
    Rbn << cos(psi), -sin(psi),
           sin(psi), cos(psi);
    //将边界点转到世界坐标系
    Eigen::Matrix<float,2,1> xn1;
    Eigen::Matrix<float,2,1> xn2;
    Eigen::Matrix<float,2,1> xn3;
    Eigen::Matrix<float,2,1> xn4;
    Eigen::Matrix<float,2,1> xn5;
    xn1 = Rbn*xb1+pos;
    xn2 = Rbn*xb2+pos;
    xn3 = Rbn*xb3+pos;
    xn4 = Rbn*xb4+pos;
    xn5 = Rbn*xb5+pos;
    //绘制模型边界
    std::vector<float> xplt1{0,0};
    std::vector<float> yplt1{0,0};
    std::vector<float> xplt2{0,0};
    std::vector<float> yplt2{0,0};
    std::vector<float> xplt3{0,0};
    std::vector<float> yplt3{0,0};
    std::vector<float> xplt4{0,0};
    std::vector<float> yplt4{0,0};
    std::vector<float> xplt5{0,0};
    std::vector<float> yplt5{0,0};

    xplt1[0] = xn1(1); 
    xplt1[1] = xn2(1); 
    yplt1[0] = xn1(0); 
    yplt1[1] = xn2(0); 

    xplt2[0] = xn2(1); 
    xplt2[1] = xn3(1); 
    yplt2[0] = xn2(0); 
    yplt2[1] = xn3(0); 

    xplt3[0] = xn3(1); 
    xplt3[1] = xn4(1); 
    yplt3[0] = xn3(0); 
    yplt3[1] = xn4(0); 

    xplt4[0] = xn4(1); 
    xplt4[1] = xn5(1); 
    yplt4[0] = xn4(0); 
    yplt4[1] = xn5(0); 

    xplt5[0] = xn5(1); 
    xplt5[1] = xn1(1); 
    yplt5[0] = xn5(0); 
    yplt5[1] = xn1(0); 


    plt::plot(xplt1,yplt1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt2,yplt2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt3,yplt3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt4,yplt4,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt5,yplt5,{{"color", model_color}, {"linestyle", "-"}});

}

void COMFUN::myWAMVplot(Matrix<float,2,1> pos, float psi, std::string model_color)
{
    //无人艇参数
    float L = 5;  // length
    float BOA = 2; //width
    float B = 1.6; //浮筒中心距离
    float Bhull = 0.4; //浮筒宽  

    //旋转矩阵
    Matrix<float,2,2> Rbn;
    Rbn << cos(psi), -sin(psi),
           sin(psi), cos(psi);

    //船体坐标系下无人艇中心船体模型边界点
    Eigen::Matrix<float,2,1> xb1;
    Eigen::Matrix<float,2,1> xb2;
    Eigen::Matrix<float,2,1> xb3;
    Eigen::Matrix<float,2,1> xb4;
    Eigen::Matrix<float,2,1> xb5;
    float Lb1 = L/6;
    xb1 << L/4-Lb1,-BOA/5;
    xb2 << L/4,0;
    xb3 << L/4-Lb1,BOA/5;
    xb4 << -L/4,BOA/5;
    xb5 << -L/4,-BOA/5;
    //将边界点转到世界坐标系
    Eigen::Matrix<float,2,1> xn1;
    Eigen::Matrix<float,2,1> xn2;
    Eigen::Matrix<float,2,1> xn3;
    Eigen::Matrix<float,2,1> xn4;
    Eigen::Matrix<float,2,1> xn5;
    xn1 = Rbn*xb1+pos;
    xn2 = Rbn*xb2+pos;
    xn3 = Rbn*xb3+pos;
    xn4 = Rbn*xb4+pos;
    xn5 = Rbn*xb5+pos;
    //绘制模型边界
    std::vector<float> xplt1{0,0};
    std::vector<float> yplt1{0,0};
    std::vector<float> xplt2{0,0};
    std::vector<float> yplt2{0,0};
    std::vector<float> xplt3{0,0};
    std::vector<float> yplt3{0,0};
    std::vector<float> xplt4{0,0};
    std::vector<float> yplt4{0,0};
    std::vector<float> xplt5{0,0};
    std::vector<float> yplt5{0,0};

    xplt1[0] = xn1(1); 
    xplt1[1] = xn2(1); 
    yplt1[0] = xn1(0); 
    yplt1[1] = xn2(0); 

    xplt2[0] = xn2(1); 
    xplt2[1] = xn3(1); 
    yplt2[0] = xn2(0); 
    yplt2[1] = xn3(0); 

    xplt3[0] = xn3(1); 
    xplt3[1] = xn4(1); 
    yplt3[0] = xn3(0); 
    yplt3[1] = xn4(0); 

    xplt4[0] = xn4(1); 
    xplt4[1] = xn5(1); 
    yplt4[0] = xn4(0); 
    yplt4[1] = xn5(0); 

    xplt5[0] = xn5(1); 
    xplt5[1] = xn1(1); 
    yplt5[0] = xn5(0); 
    yplt5[1] = xn1(0); 


    plt::plot(xplt1,yplt1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt2,yplt2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt3,yplt3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt4,yplt4,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt5,yplt5,{{"color", model_color}, {"linestyle", "-"}}); 

    //船体坐标系下无人艇浮筒模型边界点
    Eigen::Matrix<float,2,1> left_translate; //左浮筒偏移距离
    Eigen::Matrix<float,2,1> right_translate; //右浮筒偏移距离
    left_translate<< 0, -B/2;
    right_translate = -left_translate;
    //船体坐标系下无人艇无偏移浮筒模型边界点
    Eigen::Matrix<float,2,1> xb_hull1;
    Eigen::Matrix<float,2,1> xb_hull2;
    Eigen::Matrix<float,2,1> xb_hull3;
    Eigen::Matrix<float,2,1> xb_hull4;
    Eigen::Matrix<float,2,1> xb_hull5;
    float L1 = L/2;
    xb_hull1 << L/2-L1,-Bhull/2;
    xb_hull2 << L/2,0;
    xb_hull3 << L/2-L1,Bhull/2;
    xb_hull4 << -L/2,Bhull/2;
    xb_hull5 << -L/2,-Bhull/2;
    //船体坐标系下无人艇左浮筒模型边界点
    Eigen::Matrix<float,2,1> xb_left_hull1;
    Eigen::Matrix<float,2,1> xb_left_hull2;
    Eigen::Matrix<float,2,1> xb_left_hull3;
    Eigen::Matrix<float,2,1> xb_left_hull4;
    Eigen::Matrix<float,2,1> xb_left_hull5;
    xb_left_hull1 = xb_hull1+left_translate;
    xb_left_hull2 = xb_hull2+left_translate;
    xb_left_hull3 = xb_hull3+left_translate;
    xb_left_hull4 = xb_hull4+left_translate;
    xb_left_hull5 = xb_hull5+left_translate;
    //船体坐标系下无人艇右浮筒模型边界点
    Eigen::Matrix<float,2,1> xb_right_hull1;
    Eigen::Matrix<float,2,1> xb_right_hull2;
    Eigen::Matrix<float,2,1> xb_right_hull3;
    Eigen::Matrix<float,2,1> xb_right_hull4;
    Eigen::Matrix<float,2,1> xb_right_hull5;
    xb_right_hull1 = xb_hull1+right_translate;
    xb_right_hull2 = xb_hull2+right_translate;
    xb_right_hull3 = xb_hull3+right_translate;
    xb_right_hull4 = xb_hull4+right_translate;
    xb_right_hull5 = xb_hull5+right_translate;

    //将左浮筒边界点转到世界坐标系
    Eigen::Matrix<float,2,1> xn_left_hull1;
    Eigen::Matrix<float,2,1> xn_left_hull2;
    Eigen::Matrix<float,2,1> xn_left_hull3;
    Eigen::Matrix<float,2,1> xn_left_hull4;
    Eigen::Matrix<float,2,1> xn_left_hull5;
    xn_left_hull1 = Rbn*xb_left_hull1+pos;
    xn_left_hull2 = Rbn*xb_left_hull2+pos;
    xn_left_hull3 = Rbn*xb_left_hull3+pos;
    xn_left_hull4 = Rbn*xb_left_hull4+pos;
    xn_left_hull5 = Rbn*xb_left_hull5+pos;
    //将右浮筒边界点转到世界坐标系
    Eigen::Matrix<float,2,1> xn_right_hull1;
    Eigen::Matrix<float,2,1> xn_right_hull2;
    Eigen::Matrix<float,2,1> xn_right_hull3;
    Eigen::Matrix<float,2,1> xn_right_hull4;
    Eigen::Matrix<float,2,1> xn_right_hull5;
    xn_right_hull1 = Rbn*xb_right_hull1+pos;
    xn_right_hull2 = Rbn*xb_right_hull2+pos;
    xn_right_hull3 = Rbn*xb_right_hull3+pos;
    xn_right_hull4 = Rbn*xb_right_hull4+pos;
    xn_right_hull5 = Rbn*xb_right_hull5+pos;

    //绘制左浮筒模型边界
    std::vector<float> xplt_left1{0,0};
    std::vector<float> yplt_left1{0,0};
    std::vector<float> xplt_left2{0,0};
    std::vector<float> yplt_left2{0,0};
    std::vector<float> xplt_left3{0,0};
    std::vector<float> yplt_left3{0,0};
    std::vector<float> xplt_left4{0,0};
    std::vector<float> yplt_left4{0,0};
    std::vector<float> xplt_left5{0,0};
    std::vector<float> yplt_left5{0,0};

    xplt_left1[0] = xn_left_hull1(1); 
    xplt_left1[1] = xn_left_hull2(1); 
    yplt_left1[0] = xn_left_hull1(0); 
    yplt_left1[1] = xn_left_hull2(0); 

    xplt_left2[0] = xn_left_hull2(1); 
    xplt_left2[1] = xn_left_hull3(1); 
    yplt_left2[0] = xn_left_hull2(0); 
    yplt_left2[1] = xn_left_hull3(0); 

    xplt_left3[0] = xn_left_hull3(1); 
    xplt_left3[1] = xn_left_hull4(1); 
    yplt_left3[0] = xn_left_hull3(0); 
    yplt_left3[1] = xn_left_hull4(0); 

    xplt_left4[0] = xn_left_hull4(1); 
    xplt_left4[1] = xn_left_hull5(1); 
    yplt_left4[0] = xn_left_hull4(0); 
    yplt_left4[1] = xn_left_hull5(0); 

    xplt_left5[0] = xn_left_hull5(1); 
    xplt_left5[1] = xn_left_hull1(1); 
    yplt_left5[0] = xn_left_hull5(0); 
    yplt_left5[1] = xn_left_hull1(0); 


    plt::plot(xplt_left1,yplt_left1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_left2,yplt_left2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_left3,yplt_left3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_left4,yplt_left4,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_left5,yplt_left5,{{"color", model_color}, {"linestyle", "-"}}); 

    //绘制右浮筒模型边界
    std::vector<float> xplt_right1{0,0};
    std::vector<float> yplt_right1{0,0};
    std::vector<float> xplt_right2{0,0};
    std::vector<float> yplt_right2{0,0};
    std::vector<float> xplt_right3{0,0};
    std::vector<float> yplt_right3{0,0};
    std::vector<float> xplt_right4{0,0};
    std::vector<float> yplt_right4{0,0};
    std::vector<float> xplt_right5{0,0};
    std::vector<float> yplt_right5{0,0};

    xplt_right1[0] = xn_right_hull1(1); 
    xplt_right1[1] = xn_right_hull2(1); 
    yplt_right1[0] = xn_right_hull1(0); 
    yplt_right1[1] = xn_right_hull2(0); 

    xplt_right2[0] = xn_right_hull2(1); 
    xplt_right2[1] = xn_right_hull3(1); 
    yplt_right2[0] = xn_right_hull2(0); 
    yplt_right2[1] = xn_right_hull3(0); 

    xplt_right3[0] = xn_right_hull3(1); 
    xplt_right3[1] = xn_right_hull4(1); 
    yplt_right3[0] = xn_right_hull3(0); 
    yplt_right3[1] = xn_right_hull4(0); 

    xplt_right4[0] = xn_right_hull4(1); 
    xplt_right4[1] = xn_right_hull5(1); 
    yplt_right4[0] = xn_right_hull4(0); 
    yplt_right4[1] = xn_right_hull5(0); 

    xplt_right5[0] = xn_right_hull5(1); 
    xplt_right5[1] = xn_right_hull1(1); 
    yplt_right5[0] = xn_right_hull5(0); 
    yplt_right5[1] = xn_right_hull1(0); 


    plt::plot(xplt_right1,yplt_right1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_right2,yplt_right2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_right3,yplt_right3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_right4,yplt_right4,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_right5,yplt_right5,{{"color", model_color}, {"linestyle", "-"}}); 

    //计算四个横梁边界点
    //船体坐标系下无人艇无偏移横梁模型边界点
    Eigen::Matrix<float,2,1> xb_center_beam1;
    Eigen::Matrix<float,2,1> xb_center_beam2;
    Eigen::Matrix<float,2,1> xb_center_beam3;
    Eigen::Matrix<float,2,1> xb_center_beam4;
    float Lbe1 = L/20;
    float W1 = B/2-Bhull/2-BOA/5;
    xb_center_beam1 << Lbe1/2,-W1/2;
    xb_center_beam2 << Lbe1/2,W1/2;
    xb_center_beam3 << -Lbe1/2,W1/2;
    xb_center_beam4 << -Lbe1/2,-W1/2;
    //计算左1偏移横梁
    Eigen::Matrix<float,2,1> le1_translate; //偏移坐标
    Eigen::Matrix<float,2,1> xb_le1_beam1; 
    Eigen::Matrix<float,2,1> xb_le1_beam2;
    Eigen::Matrix<float,2,1> xb_le1_beam3;
    Eigen::Matrix<float,2,1> xb_le1_beam4;
    le1_translate<< 0, -W1/2-BOA/5;
    xb_le1_beam1 = xb_center_beam1+le1_translate;
    xb_le1_beam2 = xb_center_beam2+le1_translate;
    xb_le1_beam3 = xb_center_beam3+le1_translate;
    xb_le1_beam4 = xb_center_beam4+le1_translate;

    //计算左2偏移横梁
    Eigen::Matrix<float,2,1> le2_translate; //偏移坐标
    Eigen::Matrix<float,2,1> xb_le2_beam1; 
    Eigen::Matrix<float,2,1> xb_le2_beam2;
    Eigen::Matrix<float,2,1> xb_le2_beam3;
    Eigen::Matrix<float,2,1> xb_le2_beam4;
    le2_translate<< -1, -W1/2-BOA/5;
    xb_le2_beam1 = xb_center_beam1+le2_translate;
    xb_le2_beam2 = xb_center_beam2+le2_translate;
    xb_le2_beam3 = xb_center_beam3+le2_translate;
    xb_le2_beam4 = xb_center_beam4+le2_translate;  

    //计算右1偏移横梁
    Eigen::Matrix<float,2,1> rg1_translate; //偏移坐标
    Eigen::Matrix<float,2,1> xb_rg1_beam1; 
    Eigen::Matrix<float,2,1> xb_rg1_beam2;
    Eigen::Matrix<float,2,1> xb_rg1_beam3;
    Eigen::Matrix<float,2,1> xb_rg1_beam4;
    rg1_translate<< 0, W1/2+BOA/5;
    xb_rg1_beam1 = xb_center_beam1+rg1_translate;
    xb_rg1_beam2 = xb_center_beam2+rg1_translate;
    xb_rg1_beam3 = xb_center_beam3+rg1_translate;
    xb_rg1_beam4 = xb_center_beam4+rg1_translate; 

    //计算右2偏移横梁
    Eigen::Matrix<float,2,1> rg2_translate; //偏移坐标
    Eigen::Matrix<float,2,1> xb_rg2_beam1; 
    Eigen::Matrix<float,2,1> xb_rg2_beam2;
    Eigen::Matrix<float,2,1> xb_rg2_beam3;
    Eigen::Matrix<float,2,1> xb_rg2_beam4;
    rg2_translate<< -1, W1/2+BOA/5;
    xb_rg2_beam1 = xb_center_beam1+rg2_translate;
    xb_rg2_beam2 = xb_center_beam2+rg2_translate;
    xb_rg2_beam3 = xb_center_beam3+rg2_translate;
    xb_rg2_beam4 = xb_center_beam4+rg2_translate;  

    //将四个横梁边界点转到世界坐标系
    Eigen::Matrix<float,2,1> xn_le1_beam1;
    Eigen::Matrix<float,2,1> xn_le1_beam2;
    Eigen::Matrix<float,2,1> xn_le1_beam3;
    Eigen::Matrix<float,2,1> xn_le1_beam4;
    xn_le1_beam1 = Rbn*xb_le1_beam1+pos;
    xn_le1_beam2 = Rbn*xb_le1_beam2+pos;
    xn_le1_beam3 = Rbn*xb_le1_beam3+pos;
    xn_le1_beam4 = Rbn*xb_le1_beam4+pos;

    Eigen::Matrix<float,2,1> xn_le2_beam1;
    Eigen::Matrix<float,2,1> xn_le2_beam2;
    Eigen::Matrix<float,2,1> xn_le2_beam3;
    Eigen::Matrix<float,2,1> xn_le2_beam4;
    xn_le2_beam1 = Rbn*xb_le2_beam1+pos;
    xn_le2_beam2 = Rbn*xb_le2_beam2+pos;
    xn_le2_beam3 = Rbn*xb_le2_beam3+pos;
    xn_le2_beam4 = Rbn*xb_le2_beam4+pos;

    Eigen::Matrix<float,2,1> xn_rg1_beam1;
    Eigen::Matrix<float,2,1> xn_rg1_beam2;
    Eigen::Matrix<float,2,1> xn_rg1_beam3;
    Eigen::Matrix<float,2,1> xn_rg1_beam4;
    xn_rg1_beam1 = Rbn*xb_rg1_beam1+pos;
    xn_rg1_beam2 = Rbn*xb_rg1_beam2+pos;
    xn_rg1_beam3 = Rbn*xb_rg1_beam3+pos;
    xn_rg1_beam4 = Rbn*xb_rg1_beam4+pos;

    Eigen::Matrix<float,2,1> xn_rg2_beam1;
    Eigen::Matrix<float,2,1> xn_rg2_beam2;
    Eigen::Matrix<float,2,1> xn_rg2_beam3;
    Eigen::Matrix<float,2,1> xn_rg2_beam4;
    xn_rg2_beam1 = Rbn*xb_rg2_beam1+pos;
    xn_rg2_beam2 = Rbn*xb_rg2_beam2+pos;
    xn_rg2_beam3 = Rbn*xb_rg2_beam3+pos;
    xn_rg2_beam4 = Rbn*xb_rg2_beam4+pos;

    //绘制le1横梁模型边界
    std::vector<float> xplt_le1_beam1{0,0};
    std::vector<float> yplt_le1_beam1{0,0};
    std::vector<float> xplt_le1_beam2{0,0};
    std::vector<float> yplt_le1_beam2{0,0};
    std::vector<float> xplt_le1_beam3{0,0};
    std::vector<float> yplt_le1_beam3{0,0};
    std::vector<float> xplt_le1_beam4{0,0};
    std::vector<float> yplt_le1_beam4{0,0};

    xplt_le1_beam1[0] = xn_le1_beam1(1); 
    xplt_le1_beam1[1] = xn_le1_beam2(1); 
    yplt_le1_beam1[0] = xn_le1_beam1(0); 
    yplt_le1_beam1[1] = xn_le1_beam2(0); 

    xplt_le1_beam2[0] = xn_le1_beam2(1); 
    xplt_le1_beam2[1] = xn_le1_beam3(1); 
    yplt_le1_beam2[0] = xn_le1_beam2(0); 
    yplt_le1_beam2[1] = xn_le1_beam3(0); 

    xplt_le1_beam3[0] = xn_le1_beam3(1); 
    xplt_le1_beam3[1] = xn_le1_beam4(1); 
    yplt_le1_beam3[0] = xn_le1_beam3(0); 
    yplt_le1_beam3[1] = xn_le1_beam4(0); 

    xplt_le1_beam4[0] = xn_le1_beam4(1); 
    xplt_le1_beam4[1] = xn_le1_beam1(1); 
    yplt_le1_beam4[0] = xn_le1_beam4(0); 
    yplt_le1_beam4[1] = xn_le1_beam1(0); 

    plt::plot(xplt_le1_beam1,yplt_le1_beam1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_le1_beam2,yplt_le1_beam2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_le1_beam3,yplt_le1_beam3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_le1_beam4,yplt_le1_beam4,{{"color", model_color}, {"linestyle", "-"}});

    //绘制le2横梁模型边界
    std::vector<float> xplt_le2_beam1{0,0};
    std::vector<float> yplt_le2_beam1{0,0};
    std::vector<float> xplt_le2_beam2{0,0};
    std::vector<float> yplt_le2_beam2{0,0};
    std::vector<float> xplt_le2_beam3{0,0};
    std::vector<float> yplt_le2_beam3{0,0};
    std::vector<float> xplt_le2_beam4{0,0};
    std::vector<float> yplt_le2_beam4{0,0};

    xplt_le2_beam1[0] = xn_le2_beam1(1); 
    xplt_le2_beam1[1] = xn_le2_beam2(1); 
    yplt_le2_beam1[0] = xn_le2_beam1(0); 
    yplt_le2_beam1[1] = xn_le2_beam2(0); 

    xplt_le2_beam2[0] = xn_le2_beam2(1); 
    xplt_le2_beam2[1] = xn_le2_beam3(1); 
    yplt_le2_beam2[0] = xn_le2_beam2(0); 
    yplt_le2_beam2[1] = xn_le2_beam3(0); 

    xplt_le2_beam3[0] = xn_le2_beam3(1); 
    xplt_le2_beam3[1] = xn_le2_beam4(1); 
    yplt_le2_beam3[0] = xn_le2_beam3(0); 
    yplt_le2_beam3[1] = xn_le2_beam4(0); 

    xplt_le2_beam4[0] = xn_le2_beam4(1); 
    xplt_le2_beam4[1] = xn_le2_beam1(1); 
    yplt_le2_beam4[0] = xn_le2_beam4(0); 
    yplt_le2_beam4[1] = xn_le2_beam1(0); 

    plt::plot(xplt_le2_beam1,yplt_le2_beam1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_le2_beam2,yplt_le2_beam2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_le2_beam3,yplt_le2_beam3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_le2_beam4,yplt_le2_beam4,{{"color", model_color}, {"linestyle", "-"}});

    //绘制rg1横梁模型边界
    std::vector<float> xplt_rg1_beam1{0,0};
    std::vector<float> yplt_rg1_beam1{0,0};
    std::vector<float> xplt_rg1_beam2{0,0};
    std::vector<float> yplt_rg1_beam2{0,0};
    std::vector<float> xplt_rg1_beam3{0,0};
    std::vector<float> yplt_rg1_beam3{0,0};
    std::vector<float> xplt_rg1_beam4{0,0};
    std::vector<float> yplt_rg1_beam4{0,0};

    xplt_rg1_beam1[0] = xn_rg1_beam1(1); 
    xplt_rg1_beam1[1] = xn_rg1_beam2(1); 
    yplt_rg1_beam1[0] = xn_rg1_beam1(0); 
    yplt_rg1_beam1[1] = xn_rg1_beam2(0); 

    xplt_rg1_beam2[0] = xn_rg1_beam2(1); 
    xplt_rg1_beam2[1] = xn_rg1_beam3(1); 
    yplt_rg1_beam2[0] = xn_rg1_beam2(0); 
    yplt_rg1_beam2[1] = xn_rg1_beam3(0); 

    xplt_rg1_beam3[0] = xn_rg1_beam3(1); 
    xplt_rg1_beam3[1] = xn_rg1_beam4(1); 
    yplt_rg1_beam3[0] = xn_rg1_beam3(0); 
    yplt_rg1_beam3[1] = xn_rg1_beam4(0); 

    xplt_rg1_beam4[0] = xn_rg1_beam4(1); 
    xplt_rg1_beam4[1] = xn_rg1_beam1(1); 
    yplt_rg1_beam4[0] = xn_rg1_beam4(0); 
    yplt_rg1_beam4[1] = xn_rg1_beam1(0); 

    plt::plot(xplt_rg1_beam1,yplt_rg1_beam1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_rg1_beam2,yplt_rg1_beam2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_rg1_beam3,yplt_rg1_beam3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_rg1_beam4,yplt_rg1_beam4,{{"color", model_color}, {"linestyle", "-"}});

    //绘制rg2横梁模型边界
    std::vector<float> xplt_rg2_beam1{0,0};
    std::vector<float> yplt_rg2_beam1{0,0};
    std::vector<float> xplt_rg2_beam2{0,0};
    std::vector<float> yplt_rg2_beam2{0,0};
    std::vector<float> xplt_rg2_beam3{0,0};
    std::vector<float> yplt_rg2_beam3{0,0};
    std::vector<float> xplt_rg2_beam4{0,0};
    std::vector<float> yplt_rg2_beam4{0,0};

    xplt_rg2_beam1[0] = xn_rg2_beam1(1); 
    xplt_rg2_beam1[1] = xn_rg2_beam2(1); 
    yplt_rg2_beam1[0] = xn_rg2_beam1(0); 
    yplt_rg2_beam1[1] = xn_rg2_beam2(0); 

    xplt_rg2_beam2[0] = xn_rg2_beam2(1); 
    xplt_rg2_beam2[1] = xn_rg2_beam3(1); 
    yplt_rg2_beam2[0] = xn_rg2_beam2(0); 
    yplt_rg2_beam2[1] = xn_rg2_beam3(0); 

    xplt_rg2_beam3[0] = xn_rg2_beam3(1); 
    xplt_rg2_beam3[1] = xn_rg2_beam4(1); 
    yplt_rg2_beam3[0] = xn_rg2_beam3(0); 
    yplt_rg2_beam3[1] = xn_rg2_beam4(0); 

    xplt_rg2_beam4[0] = xn_rg2_beam4(1); 
    xplt_rg2_beam4[1] = xn_rg2_beam1(1); 
    yplt_rg2_beam4[0] = xn_rg2_beam4(0); 
    yplt_rg2_beam4[1] = xn_rg2_beam1(0); 

    plt::plot(xplt_rg2_beam1,yplt_rg2_beam1,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_rg2_beam2,yplt_rg2_beam2,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_rg2_beam3,yplt_rg2_beam3,{{"color", model_color}, {"linestyle", "-"}});
    plt::plot(xplt_rg2_beam4,yplt_rg2_beam4,{{"color", model_color}, {"linestyle", "-"}});
}

//白噪声生成
float COMFUN::mywhitenoise()
{
    // 生成随机种子
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    normal_distribution<double> distribution(0, 1);
    float white_noise = distribution(generator);
    return white_noise;
}
