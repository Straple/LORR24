#pragma once

#include "SharedEnv.h"

struct Position {
    int pos = 0;
    int dir = 0;// 0:east, 1:south, 2:west, 3:north

    Position() = default;

    Position(int pos, SharedEnvironment *env);

    [[nodiscard]] bool validate(SharedEnvironment *env) const;

    [[nodiscard]] Position move_forward(SharedEnvironment *env) const;
};

bool operator==(const Position &lhs, const Position &rhs);

bool operator!=(const Position &lhs, const Position &rhs);
