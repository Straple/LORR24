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

uint32_t call(const std::vector<Operation> &pool, uint32_t thr) {
    //std::cout << "call: " << std::flush;
    //Timer timer;

    {
        std::ofstream output("Tmp/actions" + std::to_string(thr) + ".txt");
        write_pool(pool, output);
    }

    // -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 10000 -t 200000 -p 100000000
    int ret_code = std::system(
            ("./cmake-build-release-wsl/lifelong -i example_problems/random.domain/random_32_32_20_300.json -o Tmp/test"//
             + std::to_string(thr)                                                                                      //
             + ".json -s 1000 -t 100 -p 100000000 --unique_id "                                                         //
             + std::to_string(thr)                                                                                      //
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
        //write_pool(pool, std::cout);
    }

    auto build_pool = [&](const std::vector<int> &state) {
        std::vector<Operation> res;
        for (uint32_t id: state) {
            res.push_back(pool[id]);
        }
        return res;
    };

    auto get_score = [&](const std::vector<int> &state, uint32_t thr) {
        return call(build_pool(state), thr);
    };

    Randomizer rnd(303);
    std::ofstream ansput("answers.txt");

    uint32_t STATES_SIZE = 36;
    std::vector<std::pair<uint32_t, std::vector<int>>> states(1);
    for (auto &[score, state]: states) {
        state = {15, 19, 20, 35};
        /*uint32_t K = rnd.get(15, 40);
        for (uint32_t k = 0; k < K; k++) {
            int x = rnd.get(0, (int) pool.size() - 1);
            if (std::find(state.begin(), state.end(), x) == state.end()) {
                state.push_back(x);
            }
        }*/
        score = get_score(state, 0);
    }
    {
        states.resize(STATES_SIZE);
        for (int i = 1; i < states.size(); i++) {
            auto &[score, state] = states[i];
            uint32_t K = rnd.get(15, 20);
            for (uint32_t k = 0; k < K; k++) {
                int x = rnd.get(0, (int) pool.size() - 1);
                if (std::find(state.begin(), state.end(), x) == state.end()) {
                    state.push_back(x);
                }
            }
            score = get_score(state, 0);
        }
        //std::vector<int> state(pool.size());
        //std::iota(state.begin(), state.end(), 0);
        //states.emplace_back(get_score(state, 0), state);
    }

    auto try_swap = [&](std::vector<int> state) {
        if (state.size() < 2) {
            return state;
        }
        std::swap(state[rnd.get(0, state.size() - 1)], state[rnd.get(0, state.size() - 1)]);
        return state;
    };

    auto try_add = [&](std::vector<int> state) {
        // 100 попыток
        for (int k = 0; k < 100; k++) {
            int x = rnd.get(0, pool.size() - 1);
            if (std::find(state.begin(), state.end(), x) == state.end()) {
                state.insert(state.begin() + rnd.get(0, state.size()), x);
                break;
            }
        }
        return state;
    };

    auto try_remove = [&](std::vector<int> state) {
        if (state.empty()) {
            return state;
        }
        int i = rnd.get(0, state.size() - 1);
        state.erase(state.begin() + i);
        return state;
    };

    constexpr uint32_t THREADS = 6;

    for (uint32_t step = 0;; step++) {
        std::cout << "NEW STEP " << step << ", " << timer << std::endl;
        std::vector<std::vector<std::pair<uint32_t, std::vector<int>>>> new_states(THREADS);
        for (const auto &[score, state]: states) {
            new_states[0].emplace_back(score, state);
        }

        auto do_work = [&](uint32_t thr) {
            for (uint32_t i = thr; i < states.size(); i += THREADS) {
                auto state = states[i].second;
                int k = rnd.get(1, 3);
                for (int i = 0; i < k; i++) {
                    double x = rnd.get_d();
                    if (x < 0.3) {
                        state = try_add(state);
                    } else if (x < 0.6) {
                        state = try_swap(state);
                    } else {
                        state = try_remove(state);
                    }
                }
                new_states[thr].emplace_back(get_score(state, thr), state);
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < THREADS; thr++) {
            threads[thr].join();
        }

        {
            states.clear();
            for (auto &data: new_states) {
                for (const auto &[score, state]: data) {
                    states.emplace_back(score, state);
                }
            }
            std::sort(states.begin(), states.end(), std::greater<>());
        }

        std::cout << "states: " << states.size() << std::endl;
        for (auto &[score, state]: states) {
            std::cout << score << " ";
        }
        std::cout << std::endl;

        if (states.size() > STATES_SIZE) {
            states.resize(STATES_SIZE);
        }

        std::cout << "score: " << states[0].first << "\n";
        write_pool(build_pool(states[0].second), ansput);
        std::cout << std::endl;
    }
}
