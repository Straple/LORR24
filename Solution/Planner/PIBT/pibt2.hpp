#pragma once

#include <Objects/Basic/position.hpp>

#include <Objects/Basic/time.hpp>

static constexpr inline uint32_t DEPTH = 3;

using Operation = std::array<Action, DEPTH>;

class BuilderActions {

    std::vector<Operation> pool;

    void generate(Operation &op, uint32_t i);

public:
    std::vector<Operation> get();
};

// 845 -> 1277 -> 1809 -> 1989 -> 2048 -> 2099 -> 2215
// 20911 -> 21819
class PIBT2 {

    // WWW -- initial
    // FWW
    // CFW
    // CCF
    // RFW

    static inline std::vector<Operation> actions = BuilderActions().get();

    /*constexpr static inline std::array<std::array<Action, DEPTH>, 11> actions = {
            {
                    {Action::W, Action::W, Action::W},

                    {Action::FW, Action::W, Action::W},
                    {Action::FW, Action::FW, Action::W},
                    {Action::FW, Action::FW, Action::FW},

                    {Action::FW, Action::CR, Action::FW},
                    {Action::FW, Action::CCR, Action::FW},

                    {Action::CR, Action::FW, Action::W},
                    {Action::CR, Action::FW, Action::FW},

                    {Action::CR, Action::CR, Action::FW},

                    {Action::CCR, Action::FW, Action::W},
                    //{Action::CCR, Action::FW, Action::CR},
                    {Action::CCR, Action::FW, Action::FW},
            }};*/

    //static inline std::vector<int32_t> actions_weight = std::vector<int32_t>(actions.size());
    //{
    //{3, 2, 1, 0, 1} // 1989
    //{3, 2, 0, 0, 0} // 1984
    //{6, 4, 3, 2, 3} //
    //{3, 0, 0, 0, 0, 0, 0, 0, 0}
    //};

    // used_edge[depth][edge] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_edge;

    // used_pos[depth][pos] = robot id
    std::array<std::vector<uint32_t>, DEPTH> used_pos;

    struct Robot {
        uint32_t start_node = 0;

        // действие, которое он хочет
        uint32_t desired = 0;
    };

    std::vector<Robot> robots;

    TimePoint end_time;

    uint32_t counter_call_build = 0;

    bool finish_time = false;

    [[nodiscard]] bool validate_path(uint32_t r) const;

    [[nodiscard]] bool check_path(uint32_t r) const;

    [[nodiscard]] std::array<uint32_t, DEPTH> get_path(uint32_t r) const;

    [[nodiscard]] uint32_t get_used(uint32_t r) const;

    void add_path(uint32_t r);

    void remove_path(uint32_t r);

    bool build(uint32_t r);

public:
    PIBT2();

    std::vector<Action> solve(const std::vector<uint32_t> &order, const TimePoint end_time);
};
