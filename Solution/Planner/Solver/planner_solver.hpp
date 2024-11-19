#pragma once

#include "../../settings.hpp"
#include "Objects/Basic/position.hpp"
#include "Objects/Basic/randomizer.hpp"
#include "solution_info.hpp"

// планирует следующие PLANNER_DEPTH шагов
class PlannerSolver {

    // обозначения:
    // r = robot idx
    // d = planner depth idx

    using Actions = std::array<Action, PLANNER_DEPTH>;

    struct Robot {
        Position start;
        int target = -1;

        // применяем действие только если оно корректно для статической карты
        // (без учета других роботов)
        // если это действие выходит за карту или переходит в препятствие
        // то мы его не выполняем
        Actions actions{Action::W};
    };

    std::vector<Robot> robots;

    double temp = 1;

    SolutionInfo solution_info;

    std::unordered_map<int, int> robot_to_idx;

    std::vector<Action> answer_actions;
    SolutionInfo answer_info;

    /* TOOLS */

    [[nodiscard]] bool is_valid(const Position& p) const;

    /* CHANGE STATE TOOLS*/

    void process_robot_path(uint32_t r, int sign);

    void add_robot_path(uint32_t r);

    void remove_robot_path(uint32_t r);

    /* ALGO TOOLS */

    bool compare(SolutionInfo old, SolutionInfo cur, Randomizer &rnd);

    template<typename rollback_t>
    bool consider(SolutionInfo old, Randomizer &rnd, rollback_t &&rollback) {
        SolutionInfo cur = get_solution_info();
        if (compare(old, cur, rnd)) {
            return true;
        } else {
            rollback();
            return false;
        }
    }

    bool try_change_robot_action(Randomizer &rnd);

    bool try_change_robot_path(Randomizer &rnd);

    bool try_change_many_robots(Randomizer &rnd);

    bool try_move_over(Randomizer &rnd);

    void init();

    void update_answer();

public:
    PlannerSolver() = default;

    PlannerSolver(std::vector<Position> robots_pos, std::vector<int> robots_target);

    void run(std::chrono::steady_clock::time_point end_time, uint64_t random_seed);

    [[nodiscard]] std::pair<SolutionInfo, std::vector<Action>> get() const;

    /* SOLUTION INFO */

    [[nodiscard]] SolutionInfo get_solution_info() const;

    [[nodiscard]] SolutionInfo get_trivial_solution_info() const;

    [[nodiscard]] double get_x(const SolutionInfo& info) const;
};
