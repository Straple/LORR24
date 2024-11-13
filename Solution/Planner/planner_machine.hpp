#pragma once

#include "../Objects/position.hpp"
#include "../Objects/randomizer.hpp"
#include "../settings.hpp"
#include "SharedEnv.h"
#include "solution_info.hpp"

// their: 359
// my: 337

// current: 220

class PlannerMachine {

    using Actions = std::array<Action, PLANNER_DEPTH>;

    static inline Actions get_wactions() {
        Actions a;
        for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
            a[k] = Action::W;
        }
        return a;
    }

    struct Robot {
        Position start;
        int target = -1;

        Actions actions = get_wactions();
    };

    std::vector<Robot> robots;

    // map[time][pos] = robot id, если в этом моменте времени на этой позиции будет стоять робот
    //                  -1, иначе
    std::array<std::vector<int>, PLANNER_DEPTH> map, map_gor, map_ver;

    SolutionInfo solution_info;

    [[nodiscard]] bool is_allowed(uint32_t k, Position p, Action action) const;

    [[nodiscard]] std::vector<uint32_t> get_collisions(uint32_t k, Position p, Action action) const;

    [[nodiscard]] double get_dist(uint32_t k, Position source, int target, double collision_power) const;

    // обновляет target у роботов
    void update_targets();

    // возвращает лучший путь для робота r без коллизий
    // [робот не должен иметь путь!]
    [[nodiscard]] std::optional<Actions> get_path(uint32_t r) const;

    [[nodiscard]] std::pair<std::vector<uint32_t>, PlannerMachine::Actions> get_collision_path(uint32_t r, double collision_power) const;

    void process_path(uint32_t r, int32_t expected, int32_t value, int32_t sign);

    // добавляет путь робота на карту
    void add_path(uint32_t r);

    // удаляет путь робота из карты
    void remove_path(uint32_t r);

    [[nodiscard]] double get_score(const SolutionInfo &info) const;

    [[nodiscard]] SolutionInfo get_solution_info() const;

    bool compare(SolutionInfo old, SolutionInfo cur, Randomizer &rnd);

    template<typename rollback_t>
    bool consider(SolutionInfo old, Randomizer &rnd, rollback_t &&rollback) {
        SolutionInfo cur = get_solution_info();
        if (compare(old, cur, rnd)) {
            return true;
        } else {
            rollback();
            ASSERT(old == get_solution_info(), "invalid rollback");
            return false;
        }
    }

    bool try_remove_and_add(Randomizer &rnd);

    bool try_crack(Randomizer &rnd);

    bool try_move_over(uint32_t k, uint32_t r, Randomizer &rnd);

    bool try_move_over(Randomizer &rnd);

public:
    explicit PlannerMachine();

    void run(TimePoint end_time);

    void set_plan(std::vector<Action> &plan) const;

    // применяет на роботах их первое действие
    // сдвигает actions
    // обновляет map
    void simulate_world();
};
