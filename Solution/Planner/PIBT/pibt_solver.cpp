#include "pibt_solver.hpp"

#include "../../Objects/environment.hpp"
#include "../../Objects/randomizer.hpp"

PIBTSolver::PIBTSolver() {
    order.resize(get_env().get_agents_size(), 0);
    iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](uint32_t lhs, uint32_t rhs) {
        return get_env().get_robot(lhs).predicted_dist < get_env().get_robot(rhs).predicted_dist;
    });
}

std::vector<Action> PIBTSolver::solve(std::chrono::steady_clock::time_point end_time) {
    Randomizer rnd;
    double best_score = 0;
    {
        PIBT pibt;
        pibt.solve(order);
        best_score = pibt.get_score();
        std::cout << best_score;
    }
    int step = 0;
    for (; std::chrono::steady_clock::now() < end_time; step++) {
        //auto new_order = order;
        //std::shuffle(new_order.begin(), new_order.end(), rnd.generator);
        uint32_t a = rnd.get(0, order.size() - 1);
        uint32_t b = rnd.get(0, order.size() - 1);
        std::swap(order[a], order[b]);

        PIBT pibt;
        pibt.solve(order);
        double new_score = pibt.get_score();
        if (new_score >= best_score) {
            if (new_score > best_score) {
                std::cout << "->" << new_score;
            }
            best_score = new_score;
        } else {
            std::swap(order[a], order[b]);
        }
    }
    std::cout << std::endl;
    std::cout << "STEPS: " << step << std::endl;
    return PIBT().solve(order);
}
