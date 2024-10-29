#include "position.hpp"

#include "../assert.hpp"

void Position::validate(SharedEnvironment *env) const {
    ASSERT(0 <= x && x < env->rows, "invalid position x: " + std::to_string(x));
    ASSERT(0 <= y && y < env->rows, "invalid position y: " + std::to_string(y));
    ASSERT(0 <= dir && dir < 4, "invalid position dir: " + std::to_string(dir));
}

Position Position::move_forward() const {
    Position p = *this;
    if (dir == 0) {
        p.y++;
    } else if (dir == 1) {
        p.x++;
    } else if (dir == 2) {
        p.y--;
    } else if (dir == 3) {
        p.x--;
    } else {
        FAILED_ASSERT("invalid position dir: " + std::to_string(dir));
    }
    return p;
}

bool operator==(const Position &lhs, const Position &rhs) {
    return lhs.x == rhs.x &&
           lhs.y == rhs.y &&
           lhs.dir == rhs.dir;
}

bool operator!=(const Position &lhs, const Position &rhs) {
    return !(lhs == rhs);
}

Position get_pos(int v, SharedEnvironment *env) {
    Position p{v / env->cols, v % env->cols, 0};
    p.validate(env);
    return p;
}
