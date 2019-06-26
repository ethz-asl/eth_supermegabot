/*
 * color.hpp
 *
 *  Created on: May 31, 2017
 *      Author: sasutosh
 */

#ifndef COLOR_HPP_
#define COLOR_HPP_

#include<ostream>

namespace Color {
    enum Code {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    };
    class Modifier {
        Code code;
    public:
        Modifier(Code pCode) : code(pCode) {}
        friend std::ostream&
        operator<<(std::ostream& os, const Modifier& mod) {
            return os << "\033[" << mod.code << "m";
        }
    };
}

Color::Modifier green_(Color::FG_GREEN);
Color::Modifier blue_(Color::FG_BLUE);
Color::Modifier red_(Color::FG_RED);
Color::Modifier def_(Color::FG_DEFAULT);



#endif /* COLOR_HPP_ */
