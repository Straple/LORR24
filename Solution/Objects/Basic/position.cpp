#include "position.hpp"

#include "Objects/Environment/map.hpp"
#include "assert.hpp"

Position::Position(uint32_t pos, uint32_t dir)
    : x(pos / get_map().get_cols()),
      y(pos % get_map().get_cols()),
      dir(dir) {
    ASSERT(0 <= dir && dir < 4, "invalid dir");
    ASSERT(0 <= pos < get_map().get_size(), "invalid pos");
    ASSERT(0 <= x && x < get_map().get_cols(), "invalid x");
    ASSERT(0 <= y && y < get_map().get_cols(), "invalid y");
}

uint32_t Position::get_pos() const {
    return x * get_map().get_cols() + y;
}

uint32_t Position::get_x() const {
    return x;
}

uint32_t Position::get_y() const {
    return y;
}

uint32_t Position::get_dir() const {
    return dir;
}

bool Position::is_valid() const {
    ASSERT(0 <= dir && dir < 4, "invalid position dir");
    return 0 <= x && x < get_map().get_rows() &&
           0 <= y && y < get_map().get_cols() &&
           get_map().is_free(get_pos());
}

Position Position::move_forward() const {
    Position p = *this;
    ASSERT(0 <= dir && dir < 4, "invalid position dir");
    if (dir == 0) {
        p.y++;
    } else if (dir == 1) {
        p.x++;
    } else if (dir == 2) {
        p.y--;
    } else if (dir == 3) {
        p.x--;
    }
    return p;
}

Position Position::rotate() const {
    Position p = *this;
    p.dir = (p.dir + 1) % 4;
    ASSERT(0 <= p.dir && p.dir < 4, "invalid position dir");
    return p;
}

Position Position::counter_rotate() const {
    Position p = *this;
    p.dir = (p.dir - 1 + 4) % 4;
    ASSERT(0 <= p.dir && p.dir < 4, "invalid position dir");
    return p;
}

Position Position::simulate_action(const Action &action) const {
    if (action == Action::FW) {
        return move_forward();
    } else if (action == Action::CR) {
        return rotate();
    } else if (action == Action::CCR) {
        return counter_rotate();
    } else {
        ASSERT(action == Action::W, "unexpected action");
        return *this;
    }
}

bool operator==(const Position &lhs, const Position &rhs) {
    return lhs.x == rhs.x &&
           lhs.y == rhs.y &&
           lhs.dir == rhs.dir;
}

bool operator!=(const Position &lhs, const Position &rhs) {
    return !(lhs == rhs);
}

bool operator<(const Position &lhs, const Position &rhs) {
    if (lhs.x != rhs.x) {
        return lhs.x < rhs.x;
    }
    if (lhs.y != rhs.y) {
        return lhs.y < rhs.y;
    }
    return lhs.dir < rhs.dir;
}
