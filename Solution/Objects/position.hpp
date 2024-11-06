#pragma once

#include "ActionModel.h"

struct Position {
    int pos = 0;
    int x = 0, y = 0;
    int dir = 0;// 0:east, 1:south, 2:west, 3:north

    Position() = default;

    Position(int pos, int dir);

    // корректная позиция + проходимо
    [[nodiscard]] bool is_valid() const;

    // двигает вперед учитывая направление
    [[nodiscard]] Position move_forward() const;

    // поворачивает по часовой стрелке
    [[nodiscard]] Position rotate() const;

    // поворачивает против часовой стрелке
    [[nodiscard]] Position counter_rotate() const;

    // применяет action на эту точку
    [[nodiscard]] Position simulate_action(Action action) const;
};

bool operator==(const Position &lhs, const Position &rhs);

bool operator!=(const Position &lhs, const Position &rhs);

bool operator<(const Position &lhs, const Position &rhs);
