//
// Created by msi on 2020/4/8.
//

#include "fundamental/vec2d.h"
#include "log/log.h"
#include <iostream>

using namespace std;
using namespace Omega::common::math;

void Vec2d_Tutorial(){
    //==============================================
    // vec2d
    //==============================================
    // 创建vec2d(向量)
    Vec2d pt(2,3);
    ADEBUG<<"点pt: "<<" x :"<<pt.x()<<" y: "<<pt.y();

    // 设置xy值
    pt.set_x(3);pt.set_y(2);
    AINFO<<"重新设置xy之后: ";
    ADEBUG<<"点pt: "<<" x :"<<pt.x()<<" y: "<<pt.y();

    // 返回向量长度
    ADEBUG<<"向量长度: "<<pt.Length();

    // 使用模长进行归一化
    pt.Normalize();
    AINFO<<"使用模长进行归一化之后: ";
    ADEBUG<<"点pt: "<<" x :"<<pt.x()<<" y: "<<pt.y();
    ADEBUG<<"向量长度: "<<pt.Length();

    // 计算到另一个点的距离 DistanceTo() DistanceSquareTo()
    Vec2d pt1(1,1);
    Vec2d pt2(2,2);
    AINFO<<"计算到另一个点的距离: ";
    ADEBUG<<"点pt1: "<<" x :"<<pt1.x()<<" y: "<<pt1.y();
    ADEBUG<<"点pt2: "<<" x :"<<pt2.x()<<" y: "<<pt2.y();
    ADEBUG<<"pt1到pt2的距离: "<<pt1.DistanceTo(pt2);

    // 计算两点的内积
    ADEBUG<<"pt1到pt2的内积: "<<pt1.InnerProd(pt2);

    // 将某个点绕原点逆时针旋转(返回新的点)
    Vec2d rot_pt1=pt1.rotate(M_PI/2);
    AINFO<<"将pt1点绕原点逆时针旋转90度: ";
    ADEBUG<<"旋转之后点rot_pt1: "<<" x :"<<rot_pt1.x()<<" y: "<<rot_pt1.y();

    // 将某个点绕原点逆时针旋转(不返回新的点)
    rot_pt1.SelfRotate(-M_PI/2);
    AINFO<<"将pt1点绕原点顺时针旋转90度: ";
    ADEBUG<<"旋转之后点rot_pt1: "<<" x :"<<rot_pt1.x()<<" y: "<<rot_pt1.y();

    // 输出某个Vec2d的信息
    ADEBUG<<pt1.DebugString();

    // 另外还有 +-*/ 等重载运算符
}

int main(int argc, char* argv[])
{
    //FLAGS_log_dir = "/home/msi/sync2/utils/utils_from_apollo/utils";
    // 设置是否输出到屏幕
    FLAGS_logtostderr=true;
    // x， 10就可以
    FLAGS_v = 10;
    google::InitGoogleLogging(argv[0]);
    cout<<"=======================fundamental============================="<<endl;
    //Vec2d_Tutorial();



    google::ShutDownCommandLineFlags();
    return 0;
}