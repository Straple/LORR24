#include "position.hpp"

#include "../assert.hpp"

Position::Position(int pos, SharedEnvironment *env) : pos(pos), dir(0) {
}

bool Position::validate(SharedEnvironment *env) const {
    ASSERT(0 <= dir && dir < 4, "invalid position dir: " + std::to_string(dir));
    return 0 <= pos && pos < env->cols * env->rows;
}

Position Position::move_forward(SharedEnvironment *env) const {
    Position p = *this;
    if (dir == 0) {
        p.pos++;
    } else if (dir == 1) {
        p.pos += env->cols;
    } else if (dir == 2) {
        p.pos--;
    } else if (dir == 3) {
        p.pos -= env->cols;
    } else {
        FAILED_ASSERT("invalid position dir: " + std::to_string(dir));
    }
    return p;
}

bool operator==(const Position &lhs, const Position &rhs) {
    return lhs.pos == rhs.pos &&
           lhs.dir == rhs.dir;
}

bool operator!=(const Position &lhs, const Position &rhs) {
    return !(lhs == rhs);
}
