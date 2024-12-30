/*
FFF
FFR
FFC
FFW
FRF
FRR
FRC
FRW
FCF
FCR
FCC
FCW
FWF
FWR
FWC
FWW 15
RFF
RFR
RFC
RFW 19
RRF 20
RRR
RRC
RRW
RCF
RCR
RCC
RCW
RWF
RWR
RWC
RWW
CFF
CFR
CFC
CFW 35
CRF
CRR
CRC
CRW
CCF
CCR
CCC
CCW
CWF
CWR
CWC
CWW
WFF
WFR
WFC
WFW
WRF
WRR
WRC
WRW
WCF
WCR
WCC
WCW
WWF
WWR
WWC
*/

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>

#include <thread>

using json = nlohmann::json;

bool verify(const Operation &op) {
    for (auto x: op) {
        if (x != Action::W) {
            return true;
        }
    }
    return false;
}

void write_pool(const std::vector<Operation> &pool, std::ostream &output) {
    output << pool.size() << '\n';
    for (auto op: pool) {
        for (auto &action: op) {
            if (action == Action::W) {
                output << 'W';
            } else if (action == Action::FW) {
                output << 'F';
            } else if (action == Action::CR) {
                output << 'R';
            } else if (action == Action::CCR) {
                output << 'C';
            } else {
                ASSERT(false, "failed");
            }
        }
        output << ' ';
    }
    output << '\n';
    output.flush();
}

void generate(Operation &op, uint32_t i, std::vector<Operation> &pool) {
    if (i == DEPTH) {
        if (verify(op)) {
            pool.push_back(op);
        }
    } else {
        for (int32_t action = 0; action < 4; action++) {
            op[i] = static_cast<Action>(action);
            generate(op, i + 1, pool);
        }
    }
}

uint32_t call(const std::string &testname, const std::vector<Operation> &pool, uint32_t thr) {
    //std::cout << "call: " << std::flush;
    //Timer timer;

    {
        std::ofstream output("Tmp/actions" + std::to_string(thr) + ".txt");
        write_pool(pool, output);
    }

    // -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 10000 -t 200000 -p 100000000
    int ret_code = std::system(
            ("./cmake-build-release-wsl/lifelong -i " + testname + " -o Tmp/test"//
             + std::to_string(thr)                                               //
             + ".json -s 1000 -t 200 -p 100000000 --unique_id "                  //
             + std::to_string(thr)                                               //
             + " > Tmp/log" + std::to_string(thr) + ".txt")
                    .c_str());

    ASSERT(ret_code == 0, "invalid ret code");

    json data;
    std::ifstream input("Tmp/test" + std::to_string(thr) + ".json");
    try {
        data = json::parse(input);
    } catch (const json::parse_error &error) {
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    uint32_t value = data["numTaskFinished"];
    //std::cout << value << ", " << timer << std::endl;
    return value;
}

int main() {
    Timer timer;
    std::vector<Operation> pool;
    {
        Operation op;
        generate(op, 0, pool);
    }

    auto build_pool = [&](const std::vector<uint32_t> &state) {
        std::vector<Operation> res;
        for (uint32_t id: state) {
            res.push_back(pool[id]);
        }
        return res;
    };

    auto get_score = [&](const std::vector<uint32_t> &state, uint32_t thr) -> uint32_t {
        auto pool = build_pool(state);
        if (call("example_problems/my.domain/random_32_32_20_100.json", pool, thr) > 350) {
            return call("example_problems/random.domain/random_32_32_20_300.json", pool, thr);
        } else {
            return 0;
        }
    };

    constexpr uint32_t THREADS = 6;
    std::vector<Randomizer> rnds(THREADS);
    {
        Randomizer rnd(20401);
        for (uint32_t i = 0; i < THREADS; i++) {
            rnds[i] = Randomizer(rnd.get());
        }
    }

    std::ofstream ansput("answers.txt");
    std::pair<uint32_t, std::vector<uint32_t>> answer(0, {});

    auto try_swap = [&](std::vector<uint32_t> state, uint32_t thr) {
        if (state.size() < 2) {
            return state;
        }
        std::swap(state[rnds[thr].get(0, state.size() - 1)], state[rnds[thr].get(0, state.size() - 1)]);
        return state;
    };

    auto try_add = [&](std::vector<uint32_t> state, uint32_t thr) {
        // 100 попыток
        for (int k = 0; k < 100; k++) {
            uint32_t x = rnds[thr].get(0, pool.size() - 1);
            if (std::find(state.begin(), state.end(), x) == state.end()) {
                state.insert(state.begin() + rnds[thr].get(0, state.size()), x);
                break;
            }
        }
        return state;
    };

    auto try_remove = [&](std::vector<uint32_t> state, uint32_t thr) {
        if (state.empty()) {
            return state;
        }
        uint32_t i = rnds[thr].get(0, state.size() - 1);
        state.erase(state.begin() + i);
        return state;
    };

    auto build_random_state = [&](uint32_t thr) {
        std::vector<uint32_t> state;
        while (true) {
            state.clear();
            uint32_t K = rnds[thr].get(5, 20);
            for (uint32_t k = 0; k < K; k++) {
                uint32_t x = rnds[thr].get(0, (int) pool.size() - 1);
                if (std::find(state.begin(), state.end(), x) == state.end()) {
                    state.push_back(x);
                }
            }

            break;
            /*auto pool = build_pool(state);
            std::set<std::pair<uint32_t, uint32_t>> S;
            for (auto operation: pool) {
                std::array<std::pair<uint32_t, uint32_t>, DEPTH> positions{};
                Position p;

                for (uint32_t d = 0; d < DEPTH; d++) {
                    p = p.simulate_action(operation[d]);
                    positions[d] = {p.get_x(), p.get_y()};
                    S.insert(positions[d]);
                }
            }

            if (S.size() >= 5) {
                break;
            }*/
        }
        return state;
    };

    for (uint32_t step = 0;; step++) {
        std::cout << "NEW STEP " << step << ", " << timer << std::endl;

        std::vector<std::pair<uint32_t, std::vector<uint32_t>>> answers(THREADS);

        auto do_work = [&](uint32_t thr) {
            Timer timer;
            auto state = build_random_state(thr);
            answers[thr] = {get_score(state, thr), state};
            while (timer.get_ms() < 100'000) {
                int k = rnds[thr].get(1, 3);
                for (int i = 0; i < k; i++) {
                    double x = rnds[thr].get_d();
                    if (x < 0.3) {
                        state = try_add(state, thr);
                    } else if (x < 0.6) {
                        state = try_swap(state, thr);
                    } else {
                        state = try_remove(state, thr);
                    }
                }
                uint32_t score = get_score(state, thr);

                if (score > answers[thr].first) {
                    answers[thr] = {score, state};
                }
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }

        std::sort(answers.begin(), answers.end(), std::greater<>());

        //std::cout << "score: " << answers[0].first << std::endl;
        std::cout << "scores: ";
        for (auto ans: answers) {
            std::cout << ans.first << ' ';
        }
        std::cout << std::endl;

        if (answer.first < answers[0].first) {
            answer = answers[0];
            ansput << "score: " << answer.first << "\n";
            write_pool(build_pool(answer.second), ansput);
            ansput << std::endl;
        }
    }
}

/*
2859:
4
CFW RCR RFC RRF

score: 2628
5
RRF FCC CFC RFR FFW

*/

/*
NEW STEP 0, 172.073us
scores: 2208 2205 736 421 222 0
NEW STEP 1, 100.782s
scores: 2294 1517 1004 918 902 0
NEW STEP 2, 212.258s
scores: 2113 1778 945 156 0 0
NEW STEP 3, 370.56s
scores: 1386 612 454 453 386 0
NEW STEP 4, 512.735s
scores: 1311 1137 0 0 0 0
NEW STEP 5, 627.635s
*/