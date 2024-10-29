#include "assert.hpp"

bool my_assert_failed(const std::string &message, const std::string filename, const int line) {
    std::cerr << "assert failed at " << filename << ":" << line << '\n';
    std::cerr << "message: \"" << message << "\"\n";
    std::exit(0);
    return true;
}