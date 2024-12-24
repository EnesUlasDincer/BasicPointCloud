// Header file (input_utils.hpp)
#ifndef funcs_HPP
#define funcs_HPP

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#endif

namespace InputUtils {

#ifdef _WIN32
inline int kbhit() {
    return _kbhit();
}

inline char getch() {
    return _getch();
}
#else
int kbhit();
char getch();
#endif

} // namespace InputUtils

#endif // funcs_HPP