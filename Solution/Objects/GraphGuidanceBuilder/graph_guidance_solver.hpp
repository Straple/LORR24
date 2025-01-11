#pragma once

#include <Objects/Basic/randomizer.hpp>
#include <Objects/Environment/graph_guidance.hpp>

#include <mutex>

class GraphGuidanceSolver {

    std::mutex mutex;

    GraphGuidance best_gg;

    int best_dhm_power = 500;

    double best_score = 0;

    void change_pointwise(GraphGuidance &gg, Randomizer &rnd) const;

    void simulate_solver(uint32_t thr);

public:
    explicit GraphGuidanceSolver(const GraphGuidance &gg);

    void solve();

    [[nodiscard]] double get_score(const GraphGuidance &gg, int dhm_power, uint32_t thr) const;
};
