//
// Created by msi on 2020/4/8.
//

#include "fundamental/vec2d.h"
#include "log/log.h"
#include <iostream>

using namespace std;
using namespace Omega::common::math;

int main(int argc, char* argv[])
{
    //FLAGS_log_dir = "/home/msi/sync2/utils/utils_from_apollo/utils";
    // 设置是否输出到屏幕
    FLAGS_logtostderr=true;
    // x， 10就可以
    FLAGS_v = 10;
    cout<<"=======================fundamental============================="<<endl;
    //=================================================
    //vec2d
    Vec2d pt(2,3);
    ADEBUG<<"点pt: "<<" x :"<<pt.x()<<" y: "<<pt.y();

    return 0;
}