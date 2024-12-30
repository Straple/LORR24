#include <settings.hpp>

uint32_t &get_unique_id() {
    static uint32_t unique_id = 0;
    return unique_id;
}