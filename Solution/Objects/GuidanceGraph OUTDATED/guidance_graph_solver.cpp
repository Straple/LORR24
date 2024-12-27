#include "guidance_graph_solver.hpp"

#include "Objects/Basic/assert.hpp"
#include "nlohmann/json.hpp"

#include <fstream>

bool GuidanceGraphSolver::compare(int64_t old, int64_t cur, Randomizer &rnd) {
    return cur >= old;
}

int64_t GuidanceGraphSolver::get_score() {
    //std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    {
        std::ofstream output("gg.txt");
        output << gg;
    }
    int ret = std::system(("./cmake-build-release-wsl/lifelong " + params + " > output.txt").c_str());
    ASSERT(ret == 0, "invalid return code: " + std::to_string(ret));
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms" << std::endl;
    //start = std::chrono::steady_clock::now();

    using json = nlohmann::basic_json<nlohmann::ordered_map>;
    json data;
    std::ifstream input("test.json");
    try {
        data = json::parse(input);
    } catch (const json::parse_error &error) {
        FAILED_ASSERT(error.what());
    }
    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << "ms" << std::endl;

    int64_t res = data["numTaskFinished"];
    if (res > best_score) {
        best_score = res;
        std::ofstream output("best_gg.txt");
        output << "score: " << res << '\n';
        output << gg;
    }
    return res;
}

bool GuidanceGraphSolver::try_change_ver_line(Randomizer &rnd) {
    int64_t old = cur_score;

    uint32_t y = rnd.get(0, COLS - 1);
    uint32_t x_left = rnd.get(0, ROWS - 1);
    uint32_t x_right = rnd.get(0, ROWS - 1);
    if (x_left > x_right) {
        std::swap(x_left, x_right);
    }
    uint32_t dir = rnd.get(0, 3);
    uint32_t action = rnd.get(0, 2);
    int8_t val = rnd.get();

    bool use2 = rnd.get() & 1;

    std::vector<uint16_t> old_vals;
    std::vector<uint16_t> old_vals_2;
    for (uint32_t x = x_left; x <= x_right; x++) {
        old_vals.push_back(gg.get(x * COLS + y, dir, action));
        gg.set(x * COLS + y, dir, action, old_vals.back() + val);

        if (use2) {
            old_vals_2.push_back(gg.get(x * COLS + y, (dir + 2) % 4, action));
            gg.set(x * COLS + y, (dir + 2) % 4, action, old_vals_2.back() - val);
        }
    }

    return consider(old, rnd, [&]() {
        for (uint32_t x = x_left; x <= x_right; x++) {
            gg.set(x * COLS + y, dir, action, old_vals[x - x_left]);
            if (use2) {
                gg.set(x * COLS + y, (dir + 2) % 4, action, old_vals_2[x - x_left]);
            }
        }
    });
}

bool GuidanceGraphSolver::try_change_gor_line(Randomizer &rnd) {
    int64_t old = cur_score;

    uint32_t x = rnd.get(0, ROWS - 1);
    uint32_t y_left = rnd.get(0, COLS - 1);
    uint32_t y_right = rnd.get(0, COLS - 1);
    if (y_left > y_right) {
        std::swap(y_left, y_right);
    }
    uint32_t dir = rnd.get(0, 3);
    uint32_t action = rnd.get(0, 2);
    int8_t val = rnd.get();

    bool use2 = rnd.get() & 1;

    std::vector<uint16_t> old_vals;
    std::vector<uint16_t> old_vals_2;
    for (uint32_t y = y_left; y <= y_right; y++) {
        old_vals.push_back(gg.get(x * COLS + y, dir, action));

        gg.set(x * COLS + y, dir, action, old_vals.back() + val);

        if (use2) {
            old_vals_2.push_back(gg.get(x * COLS + y, (dir + 2) % 4, action));
            gg.set(x * COLS + y, (dir + 2) % 4, action, old_vals_2.back() - val);
        }
    }

    return consider(old, rnd, [&]() {
        for (uint32_t y = y_left; y <= y_right; y++) {
            gg.set(x * COLS + y, dir, action, old_vals[y - y_left]);
            if (use2) {
                gg.set(x * COLS + y, (dir + 2) % 2, action, old_vals_2[y - y_left]);
            }
        }
    });
}

GuidanceGraphSolver::GuidanceGraphSolver(const GuidanceGraph &gg, const std::string &params) : gg(gg), params(params) {
}

void GuidanceGraphSolver::solve() {
    Randomizer rnd;
    cur_score = get_score();
    std::cout << "initially best: " << cur_score << std::endl;
    for (int t = 0; true; t++) {
        try_change_gor_line(rnd);
        try_change_ver_line(rnd);
    }
}