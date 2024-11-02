#pragma once

#include "ActionModel.h"
#include "SharedEnv.h"

#include "dist_machine.hpp"

class EPlanner {

    DistMachine dist_machine;

    struct Robot {
        Position position;
        Action action = Action::W;
    };

    std::vector<Robot> robots;

    // возвращает целевую точку робота или -1, если у робота нет цели (самурай)
    [[nodiscard]] int get_target(int r) const;

    // возвращает робота, который после этого шага будет стоять в pos
    [[nodiscard]] int find_robot(int pos) const;

    int move_over_best_d = 0;
    vector<pair<int, Action>> move_over_stack;
    vector<pair<int, Action>> move_over_best_stack;

    void move_over_IMPL(int r, int d);

    // выталкивает робота из его позиции, чтобы освободить место другому
    void move_over(int r);

    void plan_robot(int r);

    void flush_robots();

    /*int timestamp = 0;

    // plan_map[t][pos] = кто стоит здесь в момент времени t на позиции pos
    // либо -1, если никого
    std::map<int, std::vector<int>> plan_map;

    void remove_path(int robot);

    void build_path(int robot, int target);*/

public:
    SharedEnvironment *env;

    explicit EPlanner(SharedEnvironment *env);
    EPlanner();

    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> &plan);
};
