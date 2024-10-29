#pragma once

#include <iostream>
#include <string>

///////// !!!
#define MY_DEBUG_MODE
///////// !!!

bool my_assert_failed(const std::string &message, const std::string filename, const int line) {
    std::cerr << "assert failed at " << filename << ":" << line << '\n';
    std::cerr << "message: \"" << message << "\"\n";
    std::exit(0);
    return true;
}

#ifdef MY_DEBUG_MODE

#define FAILED_ASSERT(message) my_assert_failed((message), __FILE__, __LINE__)

#define ASSERT(condition, message) (!(condition) && FAILED_ASSERT(message))

#else

#define ASSERT(condition, message) /*condition*/ /**THIS CONDITION VERY IMPORTANT */

#endif// MY_DEBUG_MODE