#include "position.hpp"

#include "../assert.hpp"
#include "environment.hpp"

Position::Position(int pos, int dir) : pos(pos), x(pos / get_env().get_cols()), y(pos % get_env().get_cols()),
                                       dir(dir) {
}

bool Position::is_valid() const {
    ASSERT(0 <= dir && dir < 4, "invalid position dir: " + std::to_string(dir));
    return 0 <= x && x < get_env().get_rows() &&
           0 <= y && y < get_env().get_cols() &&
           0 <= pos && pos < get_env().get_rows() * get_env().get_cols() &&
           get_env().is_free(pos);
}

Position Position::move_forward() const {
    Position p = *this;
    if (dir == 0) {
        p.y++;
        p.pos++;
    } else if (dir == 1) {
        p.pos += get_env().get_cols();
        p.x++;
    } else if (dir == 2) {
        p.pos--;
        p.y--;
    } else if (dir == 3) {
        p.pos -= get_env().get_cols();
        p.x--;
    } else {
        FAILED_ASSERT("invalid position dir: " + std::to_string(dir));
    }
    return p;
}

Position Position::rotate() const {
    Position p = *this;
    p.dir = (p.dir + 1) % 4;
    return p;
}

Position Position::counter_rotate() const {
    Position p = *this;
    p.dir = (p.dir - 1 + 4) % 4;
    return p;
}

[[nodiscard]] Position Position::simulate_action(Action action) const {
    if (action == Action::FW) {
        return move_forward();
    } else if (action == Action::CR) {
        return rotate();
    } else if (action == Action::CCR) {
        return counter_rotate();
    } else {
        return *this;
    }
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
