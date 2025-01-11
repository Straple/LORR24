#include <Objects/GraphGuidanceBuilder/graph_guidance_solver.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>
#include <Objects/Basic/position.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <thread>

//finish(0): 12188.2, 2460, 67.5548s
//finish(4): 12188.2, 2460, 100.691s
//improve(4): 12188.2 -> 12188.2
//finish(2): 12188.2, 2460, 100.914s
//improve(2): 12188.2 -> 12188.2
//finish(5): 12188.2, 2460, 101.498s
//improve(5): 12188.2 -> 12188.2
//finish(0): 12188.2, 2460, 101.683s
//improve(0): 12188.2 -> 12188.2
//finish(3): 12188.2, 2460, 101.753s
//improve(3): 12188.2 -> 12188.2
//finish(1): 12188.2, 2460, 101.913s
//improve(1): 12188.2 -> 12188.2
//finish(4): 12188.2, 2460, 106.755s
//improve(4): 12188.2 -> 12188.2
//finish(2): 12188.2, 2460, 106.812s
//improve(2): 12188.2 -> 12188.2
//finish(5): 12188.2, 2460, 107.531s
//improve(5): 12188.2 -> 12188.2
//finish(1): 0, 2460, 107.686s
//finish(3): 12188.2, 2460, 107.908s
//improve(3): 12188.2 -> 12188.2
//finish(0): 12188.2, 2460, 108.65s
//improve(0): 12188.2 -> 12188.2
//finish(1): 12188.2, 2460, 111.307s
//improve(1): 12188.2 -> 12188.2
//finish(4): 12188.2, 2460, 113.655s
//improve(4): 12188.2 -> 12188.2
//finish(3): 0, 2460, 111.65s
//finish(2): 12188.2, 2460, 113.637s
//improve(2): 12188.2 -> 12188.2
//finish(0): 12188.2, 2460, 111.468s
//improve(0): 12188.2 -> 12188.2
//finish(5): 12188.2, 2460, 113.77s
//improve(5): 12188.2 -> 12188.2
//finish(4): 12188.2, 2460, 107.867s
//improve(4): 12188.2 -> 12188.2
//finish(2): 12188.2, 2460, 107.819s
//improve(2): 12188.2 -> 12188.2
//finish(3): 8636.43, 2460, 108.683s
//finish(0): 8636.43, 2567, 108.203s
//finish(1): 8636.43, 2460, 109.386s
//finish(5): 8636.43, 2567, 108.48s
//finish(4): 8636.43, 2567, 103.044s
//finish(2): 8636.43, 2567, 103.784s
//finish(0): 11740.5, 2426, 103.126s
//finish(3): 11740.5, 2426, 104.07s
//finish(1): 11740.5, 2426, 104.45s
//finish(5): 11740.5, 2426, 104.579s

void GraphGuidanceSolver::change_pointwise(GraphGuidance &gg, Randomizer &rnd) const {
    uint32_t dir = rnd.get(0, 3);
    uint32_t act = rnd.get(0, 3);
    uint32_t pos = rnd.get(1, gg.get_size() - 1);
    uint32_t len = rnd.get(1, 20);

    int diff = rnd.get(-10, 10);
    while (len--) {
        int val = gg.get(pos, dir, act);
        val += diff;
        val = std::max(100, val);
        val = std::min(10'000, val);
        gg.set(pos, dir, act, val);

        int x = (pos - 1) / gg.get_cols();
        int y = (pos - 1) % gg.get_rows();
        ASSERT(0 <= x && x < gg.get_rows(), "invalid x");
        ASSERT(0 <= y && y < gg.get_cols(), "invalid y");

        int dir = rnd.get(0, 3);
        if (dir == 0) {
            y++;
        } else if (dir == 1) {
            x++;
        } else if (dir == 2) {
            y--;
        } else if (dir == 3) {
            x--;
        }

        if (0 <= x && x < gg.get_rows() && 0 <= y && y < gg.get_cols()) {
            pos = x * gg.get_cols() + y + 1;
        }
    }
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
        ASSERT(std::abs(cur_score - again_score) < 1e-9,
               "invalid scores: " + std::to_string(cur_score) + " != " + std::to_string(again_score) + ", thr: " +
               std::to_string(thr));

        {
            std::unique_lock locker(mutex);
            if (cur_score <= best_score) {
                Printer() << "improve(" << thr << "): " << best_score << " -> " << cur_score << '\n';
                Printer().get().flush();

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

GraphGuidanceSolver::GraphGuidanceSolver(const GraphGuidance &gg, int dhm_power)
        : best_gg(gg),
          best_dhm_power(dhm_power),
          best_score(get_score(gg, best_dhm_power, 0)) {
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

double GraphGuidanceSolver::get_score(const GraphGuidance &gg, int dhm_power, uint32_t thr) {
    Timer timer;
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
             std::to_string(thr) + ".json -s 200 -t 100000000 -p 100000000 --unique_id " + std::to_string(thr) +
             " > Tmp/output" +
             std::to_string(thr)).c_str());
    ASSERT(ret == 0, "invalid return code: " + std::to_string(ret));

    uint32_t finished_tasks = 0;
    {
        using json = nlohmann::basic_json<nlohmann::ordered_map>;
        json data;
        std::ifstream input("Tmp/test" + std::to_string(thr) + ".json");
        try {
            data = json::parse(input);
        } catch (const json::parse_error &error) {
            FAILED_ASSERT(error.what());
        }
        finished_tasks = data["numTaskFinished"];
    }

    std::ifstream input("Tmp/meta" + std::to_string(thr));
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
    score -= finished_tasks;

    {
        std::unique_lock locker(mutex);
        Printer() << "finish(" << thr << "): " << score << ", " << finished_tasks << ", " << timer << '\n';
        Printer().get().flush();
    }
    return score;
}
