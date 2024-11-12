#pragma once

#include "../Objects/position.hpp"
#include "../Objects/randomizer.hpp"
#include "../settings.hpp"
#include "SharedEnv.h"

// their: 359
// my: 337

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

    [[nodiscard]] bool is_allowed(int k, Position p, Action action) const;

    // обновляет target у роботов
    void update_targets();

    // возвращает лучший путь для робота r без коллизий
    // [робот не должен иметь путь!]
    [[nodiscard]] std::optional<Actions> get_path(uint32_t r) const;

    // добавляет путь робота на карту
    void add_path(uint32_t r);

    // удаляет путь робота из карты
    void remove_path(uint32_t r);

    void try_remove_and_add(Randomizer& rnd);

public:
    explicit PlannerMachine();

    void run(TimePoint end_time);

    void set_plan(std::vector<Action> &plan) const;

    // применяет на роботах их первое действие
    // сдвигает actions
    // обновляет map
    void simulate_world();
};
