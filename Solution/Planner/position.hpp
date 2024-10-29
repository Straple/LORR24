#pragma once

#include "SharedEnv.h"

struct Position {
    int x = 0, y = 0;

    // 0:east, 1:south, 2:west, 3:north
    int dir = 0;

    void validate(SharedEnvironment *env) const;

    [[nodiscard]] Position move_forward() const;
};

bool operator==(const Position &lhs, const Position &rhs);

bool operator!=(const Position &lhs, const Position &rhs);

Position get_pos(int v, SharedEnvironment *env);
