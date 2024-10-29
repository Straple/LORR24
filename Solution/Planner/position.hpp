#pragma once

#include "SharedEnv.h"

struct Position {
    int pos = 0;
    int dir = 0;// 0:east, 1:south, 2:west, 3:north

    Position() = default;

    Position(int pos, SharedEnvironment *env);

    // корректная позиция + проходимо
    [[nodiscard]] bool is_valide(SharedEnvironment *env) const;

    // двигает вперед учитывая направление
    [[nodiscard]] Position move_forward(SharedEnvironment *env) const;

    // поворачивает по часовой стрелке
    [[nodiscard]] Position rotate() const;

    // поворачивает против часовой стрелке
    [[nodiscard]] Position counter_rotate() const;
};

bool operator==(const Position &lhs, const Position &rhs);

bool operator!=(const Position &lhs, const Position &rhs);

bool operator<(const Position &lhs, const Position &rhs);