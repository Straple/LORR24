#pragma once

#include <string>

#include <settings.hpp>

bool my_assert_failed(const std::string &message, const std::string &filename, const int line);

#ifdef ENABLE_ASSERT

#define FAILED_ASSERT(message) my_assert_failed((message), __FILE__, __LINE__)

#define ASSERT(condition, message) (!(condition) && FAILED_ASSERT(message))

#else

#define FAILED_ASSERT(message) (false)

#define ASSERT(condition, message) (false) /*condition*/ /**THIS CONDITION VERY IMPORTANT */

#endif// ENABLE_ASSERT
