#pragma once

#include <ActionModel.h>

#include <array>
#include <cstdint>
#include <vector>

static constexpr inline uint32_t DEPTH = 3;

using Operation = std::array<Action, DEPTH>;

using EPath = std::array<uint32_t, DEPTH>;

class OperationsGenerator {
    std::vector<Operation> pool;

    void generate(Operation &op, uint32_t i);

public:
    std::vector<Operation> get();
};

std::vector<Operation> &get_operations();
