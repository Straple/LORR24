#include "position.hpp"

#include "../assert.hpp"

Position::Position(int pos, int dir, SharedEnvironment *env) : pos(pos), x(pos / env->cols), y(pos % env->cols), dir(dir) {
}

bool Position::is_valide(SharedEnvironment *env) const {
    ASSERT(0 <= dir && dir < 4, "invalid position dir: " + std::to_string(dir));
    return 0 <= x && x < env->rows &&
           0 <= y && y < env->cols &&
           0 <= pos && pos < env->cols * env->rows &&
           env->map[pos] == 0;
}

Position Position::move_forward(SharedEnvironment *env) const {
    Position p = *this;
    if (dir == 0) {
        p.y++;
        p.pos++;
    } else if (dir == 1) {
        p.pos += env->cols;
        p.x++;
    } else if (dir == 2) {
        p.pos--;
        p.y--;
    } else if (dir == 3) {
        p.pos -= env->cols;
        p.x--;
    } else {
        FAILED_ASSERT("invalid position dir: " + std::to_string(dir));
    }
    return p;
}

// поворачивает по часовой стрелке
Position Position::rotate() const {
    Position p = *this;
    p.dir = (p.dir + 1) % 4;
    return p;
}

// поворачивает против часовой стрелке
Position Position::counter_rotate() const {
    Position p = *this;
    p.dir = (p.dir - 1 + 4) % 4;
    return p;
}

bool operator==(const Position &lhs, const Position &rhs) {
    return lhs.x == rhs.x &&
           lhs.y == rhs.y &&
           lhs.pos == rhs.pos &&
           lhs.dir == rhs.dir;
}

bool operator!=(const Position &lhs, const Position &rhs) {
    return !(lhs == rhs);
}

bool operator<(const Position &lhs, const Position &rhs) {
    if (lhs.pos != rhs.pos) {
        return lhs.pos < rhs.pos;
    }
    return lhs.dir < rhs.dir;
}