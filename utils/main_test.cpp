//
// Created by msi on 2020/4/8.
//

#include "vec2d.h"
#include "vec3d.h"
#include "search.h"
#include "matrix_operations.h"
#include "math_utils.h"
#include "integral.h"
#include "transform/euler_angles_zxy.h"
#include "transform/quaternion_zxy.h"
#include "log/log.h"
#include <iostream>

using namespace std;
using namespace Omega::common::math;

//===========for `search` and `integral` Tutorial====================
double LinearFunc(double x)
{ return 2.0 * x; }

double SquareFunc(double x)
{ return x * x; }

double CubicFunc(double x)
{ return (x - 1.0) * (x - 2.0) * (x - 3.0); }

double SinFunc(double x)
{ return std::sin(x); }
//==================================================

void Vec2d_Tutorial()
{
    //==============================================
    // vec2d
    //==============================================
    // 创建vec2d(向量)
    Vec2d pt(2, 3);
    ADEBUG << "点pt: " << " x :" << pt.x() << " y: " << pt.y();

    // 设置xy值
    pt.setX(3);
    pt.setY(2);
    AINFO << "重新设置xy之后: ";
    ADEBUG << "点pt: " << " x :" << pt.x() << " y: " << pt.y();

    // 返回向量长度
    ADEBUG << "向量长度: " << pt.norm();

    // 使用模长进行归一化
    pt.normalize();
    AINFO << "使用模长进行归一化之后: ";
    ADEBUG << "点pt: " << " x :" << pt.x() << " y: " << pt.y();
    ADEBUG << "向量长度: " << pt.norm();

    // 计算到另一个点的距离 DistanceTo() DistanceSquareTo()
    Vec2d pt1(1, 1);
    Vec2d pt2(2, 2);
    AINFO << "计算到另一个点的距离: ";
    ADEBUG << "点pt1: " << " x :" << pt1.x() << " y: " << pt1.y();
    ADEBUG << "点pt2: " << " x :" << pt2.x() << " y: " << pt2.y();
    ADEBUG << "pt1到pt2的距离: " << pt1.distanceTo(pt2);

    // 计算两点的内积
    ADEBUG << "pt1到pt2的内积: " << pt1.dot(pt2);

    // 按元素相乘
    ADEBUG << "pt1和pt2按元素相乘，返回向量: "<< pt1.cwiseProduct(pt2).transpose();

    // 输出某个Vec2d的信息
    ADEBUG << pt1.DebugString();

}

void Vec3d_Tutorial()
{
    //==============================================
    // vec3d
    //==============================================
    // 创建vec3d(向量)
    Vec3d pt(2, 3,1);
    ADEBUG << pt.DebugString();

    // 设置xy值
    pt.setX(3);
    pt.setY(2);
    pt.setZ(1);
    AINFO << "重新设置xy之后: ";
    ADEBUG << pt.DebugString();

    // 返回向量长度
    ADEBUG << "向量长度: " << pt.norm();

    // 使用模长进行归一化
    pt.normalize();
    AINFO << "使用模长进行归一化之后: ";
    ADEBUG << pt.DebugString();
    ADEBUG << "向量长度: " << pt.norm();

    // 计算到另一个点的距离 DistanceTo() DistanceSquareTo()
    Vec3d pt1(1, 1,1);
    Vec3d pt2(2, 2,2);
    AINFO << "计算到另一个点的距离: ";
    ADEBUG << "点pt1: " << pt1.DebugString();
    ADEBUG << "点pt2: " << pt2.DebugString();
    ADEBUG << "pt1到pt2的距离: " << pt1.distanceTo(pt2);

    // 计算两点的内积
    ADEBUG << "pt1到pt2的内积: " << pt1.dot(pt2);

    // 按元素相乘
    ADEBUG << "pt1和pt2按元素相乘，返回向量: "<< pt1.cwiseProduct(pt2).transpose();

}

void search_Tutoral()
{
    // 找最(大，小)值
    /// GoldenSectionSearch(函数，搜索下界，搜索上界，误差接受范围)
    double linear_argmin = GoldenSectionSearch(LinearFunc, 0.0, 1.0, 1e-6);
    ADEBUG << "linear_argmin: " << linear_argmin;
    double square_argmin = GoldenSectionSearch(SquareFunc, -1.0, 2.0, 1e-6);
    ADEBUG << "square_argmin: " << square_argmin;
    double cubic_argmin_1 = GoldenSectionSearch(CubicFunc, 0.0, 1.5, 1e-6);
    ADEBUG << "cubic_argmin_1: " << cubic_argmin_1;
    double cubic_argmin_2 = GoldenSectionSearch(CubicFunc, 1.0, 1.8, 1e-6);
    ADEBUG << "cubic_argmin_2: " << cubic_argmin_2;
    double cubic_argmin_3 = GoldenSectionSearch(CubicFunc, 2.0, 3.0, 1e-6);
    ADEBUG << "cubic_argmin_3: " << cubic_argmin_3;
    double sin_argmin = GoldenSectionSearch(SinFunc, 0.0, 2 * M_PI, 1e-6);
    ADEBUG << "sin_argmin: " << sin_argmin;
}

void math_utils_Tutorial()
{
    /**
     * 1. 计算平方
     * 2. 将角度限制在[0,2pi]
     * 3. 求两个角度的差值
     * 4. 随机生成INT数
     * 5. 随机生成Double数
     * 6. 从高斯分布中采样一个数
     * 7. sigmoid函数
     * 8. 设置边界值
     * 9. 输入笛卡尔直角坐标系，转换成极坐标
     */

    // 计算平方
    ADEBUG << "4 的平方: " << Sqr(4);

    // 角度处理相关
    ADEBUG << "将`361`度 规范化: " << RadToDeg(WrapAngle(DegToRad(361)));
    ADEBUG << "将`-36`度 规范化: " << RadToDeg(WrapAngle(DegToRad(-36)));
    ADEBUG << "将`720`度 规范化: " << RadToDeg(WrapAngle(DegToRad(720)));

    ADEBUG << "求 -5度 和 5度 的差值: " << AngleDiff(-5 * M_PI / 180, 5 * M_PI / 180) * 180 / M_PI;

    // 生成随机数 (下限，上限，随机数种子)
    ADEBUG << "随机生成INT " << RandomInt(10, 20, (int) time(0));

    ADEBUG << "随机生成Double " << RandomDouble(10.5, 20.5, (int) time(0));

    // 高斯分布
    ADEBUG << "高斯分布: mean 0 std 10 x= ..." << Gaussian(0, 10, 5);

    // Sigmod函数
    ADEBUG << "Sigmoid 0=" << Sigmoid(0);

    // 将某个类型(int，double等)，限制在范围内
    ADEBUG << "把5.5 限制在[0,5]之间 ==>" << Clamp<double>(5.5, 0, 5);

//    // 直角坐标转 极坐标
//    ADEBUG << "(1,1) 转成极坐标: " << Vec2d(Cartesian2Polar(1, 1).first, Cartesian2Polar(1, 1).second).DebugString();

}

void integral_Tutorial()
{

    /// 使用GaussLegendre积分
    double linear_integral = IntegrateByGaussLegendre<5>(LinearFunc, 0.0, 2.0);
    ADEBUG << "使用GaussLegendre 对线性函数f=2x积分 从[0,2]: " << linear_integral;
    //EXPECT_NEAR(linear_integral, 1.0, 1e-5);

    double square_integral = IntegrateByGaussLegendre<5>(SquareFunc, 0.0, 1.0);
    ADEBUG << "使用GaussLegendre 对x^2函数积分 从[0,1]: " << square_integral;
    //EXPECT_NEAR(square_integral, 1.0 / 3.0, 1e-5);

    double cubic_integral = IntegrateByGaussLegendre<5>(CubicFunc, 0.0, 1.0);
    ADEBUG << "使用GaussLegendre 对x^3函数积分 从[0,1]: " << cubic_integral;
    //EXPECT_NEAR(cubic_integral, 1.0 / 4.0, 1e-5);

    double sin_integral = IntegrateByGaussLegendre<5>(SinFunc, 0.0, 0.5 * M_PI);
    ADEBUG << "使用GaussLegendre 对sin函数积分 从[0,pi]: " << sin_integral;
    //EXPECT_NEAR(sin_integral, 1.0, 1e-5);

}

void zxy_312_euler_Tutorial()
{
    // * 对应`右-前-上` 的车体坐标系 x-y-z
    // * 1) 俯仰角pitch：(-pi/2, pi/2) 绕x轴旋转， 车头翘起来则 pitch>0       (逆时针为正)
    // * 2) 横滚角roll:[-pi, pi) 绕y轴旋转，从车尾向车头看，左侧翘起来，则roll>0 (逆时针为正)
    // * 3) 偏航角yaw:[-pi, pi) 绕z轴转，指向西边为正(北偏西)                  (逆时针为正)
    // *
    // * 对应于东北天E-N-U导航坐标系
    // * 这些角度，代表了从 ENU导航坐标系 转换到车体坐标系的变换

    EulerAnglesZXYd euler_(0, 0, 0);

    // zxy四元数
    Eigen::Quaterniond q_zxy = euler_.ToQuaternion();

    // 求此时的车头朝向
    ADEBUG << "zxy欧拉角是: [x]pitch: " << euler_.pitch() << " [y]roll: " << euler_.roll() << " [z]yaw: " << euler_.yaw();
    ADEBUG << "Heading is: " << Quaternion_zxyToHeading(q_zxy);
}

int main(int argc, char *argv[])
{
    //============================Init Log=======================================
    //FLAGS_log_dir = "/home/msi/sync2/utils/utils_from_apollo/utils";
    // 设置是否输出到屏幕
    FLAGS_logtostderr = true;
    // x， 10就可以
    FLAGS_v = 10;
    google::InitGoogleLogging(argv[0]);

    cout << "=======================type_def=============================" << endl;

    cout << "===1. Vec2d===" << endl;
    //Vec2d_Tutorial();

    cout << "===2. Vec3d===" << endl;
    //Vec3d_Tutorial();


    cout << "===6. zxy Euler Angle + quaternion_zxy===" << endl;
    //zxy_312_euler_Tutorial();

    cout << "=======================fundamental=============================" << endl;

    cout << "===1. search===" << endl;
    //search_Tutoral();

    cout << "===2. matrix High Level Operation===" << endl;
    //just read the file "matrix_operations.h"

    cout << "===3. math_utils===" << endl;
    //math_utils_Tutorial();

    cout << "===4. integral===" << endl;
    //integral_Tutorial();

    google::ShutDownCommandLineFlags();
    return 0;
}