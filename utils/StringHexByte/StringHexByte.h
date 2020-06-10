//
// Created by msi on 2020/6/10.
//

#ifndef OMEGA_UTILS_STRINGHEXBYTE_H
#define OMEGA_UTILS_STRINGHEXBYTE_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <sstream>

namespace Omega {
namespace common {

    class NumberConversionError : public std::exception {
    public:
        std::string message;
        NumberConversionError(std::string msg)
                : message(msg)
        {};

        virtual ~NumberConversionError()
        {};

        std::string what(){
            return message;
        }
    };

    // string转double
    double string2Double(std::string s){
        char* p;
        //strtod函数返回转换后的双精度浮点数，如果没有执行有效的转换，则返回零（0.0）
        double d = ::strtod(s.c_str(), &p);
        if (*p != 0){
            std::stringstream ss;
            ss << "NumberConversionError: parseDouble() error in argument \"" << s << "\", '"
               << *p << "' is not a number.";
            throw NumberConversionError(ss.str());
        }
        return d;
    }

    // string转INT
    // parseInt("4B",16) ====> 75 [如果输出为16进制，刚好是 0x4b]
    // parseInt("10",16) ====> 16
    // parseInt("10",10) ====> 10
    int64_t string2IntOrHex(std::string s, int radix){
        char* p;

        int64_t d = ::strtoll(s.c_str(), &p, radix);

        if (*p != 0) {
            std::stringstream ss;
            ss << "NumberConversionError: parseInt() error in argument \"" << s << "\", '"
               << *p << "' is not a number.";
            throw NumberConversionError(ss.str());
        }
        return d;
    }

}
}
#endif //OMEGA_UTILS_STRINGHEXBYTE_H
