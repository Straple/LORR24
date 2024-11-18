#pragma once

#include "../../Objects/position.hpp"
#include "../../settings.hpp"

class PIBTStar {
    using Actions = std::array<Action, PLANNER_DEPTH>;

    static inline Actions get_w_actions() {
        Actions result;
        for (uint32_t i = 0; i < PLANNER_DEPTH; i++) {
            result[i] = Action::W;
        }
        return result;
    }

    struct Robot {
        Position p;

        Actions actions = get_w_actions();

        int target = -1;

        int64_t priority = 0;

        bool is_phantom = true;

        bool is_done = false;
    };

    std::vector<Robot> robots;

    std::array<std::vector<int>, PLANNER_DEPTH> map, map_gor, map_ver;

    struct Operation {
        uint32_t r = 0;
        Actions actions = get_w_actions();
        enum Type {
            ADD,
            REMOVE,
            SET,
        } type = ADD;
    };
    std::vector<Operation> stack;

    void rollback();

    void rollback(uint32_t to_size);

    void add_path_IMPL(uint32_t r);

    void remove_path_IMPL(uint32_t r);

    void set_path(uint32_t r, Actions actions);

    void add_path(uint32_t r);

    void remove_path(uint32_t r);

    bool build(uint32_t r);

public:
    PIBTStar();

    std::vector<Action> solve(const std::vector<uint32_t> &order);
};