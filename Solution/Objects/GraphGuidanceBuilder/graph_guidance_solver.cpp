#include <Objects/GraphGuidanceBuilder/graph_guidance_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <thread>

void GraphGuidanceSolver::change_pointwise(GraphGuidance &gg, Randomizer &rnd) const {
    uint32_t dir = rnd.get(0, 3);
    uint32_t act = rnd.get(0, 3);
    uint32_t pos = rnd.get(1, gg.get_size() - 1);

    int val = gg.get(pos, dir, act);
    val += rnd.get(-10, 10);
    val = std::max(100, val);
    val = std::min(10'000, val);
    gg.set(pos, dir, act, val);
}

void GraphGuidanceSolver::simulate_solver(uint32_t thr) {
    Randomizer rnd(42 * thr + 5001);
    while (true) {
        GraphGuidance gg;
        int dhm_power;
        {
            std::unique_lock locker(mutex);
            gg = best_gg;
            dhm_power = best_dhm_power;
        }

        if (rnd.get_d() < 0.9) {
            change_pointwise(gg, rnd);
        } else {
            dhm_power += rnd.get(-5, 5);
            dhm_power = std::max(100, dhm_power);
            dhm_power = std::min(10'000, dhm_power);
        }

        double cur_score = get_score(gg, dhm_power, thr);
        double again_score = get_score(gg, dhm_power, thr);
        ASSERT(std::abs(cur_score - again_score) < 1e-9, "invalid scores");

        {
            std::unique_lock locker(mutex);
            if (cur_score >= best_score) {
                Printer() << "improve(" << thr << "): " << best_score << " -> " << cur_score << '\n';
                best_score = cur_score;
                best_gg = gg;
                best_dhm_power = dhm_power;

                {
                    std::ofstream output("best_gg");
                    output << best_gg;
                }
                {
                    std::ofstream output("best_args");
                    output << best_dhm_power;
                }
            }
        }
    }
}

GraphGuidanceSolver::GraphGuidanceSolver(const GraphGuidance &gg) : best_gg(gg), best_score(get_score(gg, best_dhm_power, 0)) {
}

void GraphGuidanceSolver::solve() {
    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread([&](uint32_t thr) {
            simulate_solver(thr);
        }, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }
}

double GraphGuidanceSolver::get_score(const GraphGuidance &gg, int dhm_power, uint32_t thr) const {
    {
        std::ofstream output("Tmp/gg" + std::to_string(thr));
        output << gg;
    }
    {
        std::ofstream output("Tmp/args" + std::to_string(thr));
        output << dhm_power;
    }
    int ret = std::system(
            ("./cmake-build-release-wsl/lifelong -i ./example_problems/random.domain/random_32_32_20_300.json -o Tmp/test" +
             std::to_string(thr) + ".json -s 2000 -t 100000000 -p 100000000 > Tmp/output" +
             std::to_string(thr)).c_str());
    ASSERT(ret == 0, "invalid return code: " + std::to_string(ret));

    /*using json = nlohmann::basic_json<nlohmann::ordered_map>;
    json data;
    std::ifstream input("Tmp/test" + std::to_string(thr) + ".json");
    try {
        data = json::parse(input);
    } catch (const json::parse_error &error) {
        FAILED_ASSERT(error.what());
    }
    int64_t res = data["numTaskFinished"];*/

    std::ifstream input("Tmp/meta" + std::to_string(get_unique_id()));
    // [pos][dir][action]
    std::vector<std::array<std::array<uint32_t, 5>, 5>> dp(gg.get_size());

    uint32_t rows, cols;
    input >> rows >> cols;

    ASSERT(gg.get_rows() == rows && gg.get_cols() == cols, "invalid rows/cols");
    for (uint32_t dir = 0; dir < 5; dir++) {
        for (uint32_t act = 0; act < 5; act++) {
            for (uint32_t pos = 0; pos < gg.get_size(); pos++) {
                input >> dp[pos][dir][act];
            }
        }
    }

    auto calc = [&](uint32_t dir, uint32_t act) {
        uint64_t s = 0;
        for (uint32_t pos = 0; pos < gg.get_size(); pos++) {
            s += static_cast<uint64_t>(dp[pos][dir][act]) * dp[pos][dir][act];
        }
        return static_cast<double>(s) / (gg.get_size() - 1);
    };

    double score = 0;
    for (uint32_t dir = 0; dir < 5; dir++) {
        score += calc(dir, 1);
        score += calc(dir, 2);
        score += calc(dir, 3) * 3;
    }
    return score;
}
