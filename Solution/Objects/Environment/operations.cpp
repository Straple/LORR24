#include <Objects/Environment/operations.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>

#include <fstream>

bool verify_lol(const Operation &op) {
    return true;
}

void OperationsGenerator::generate(Operation &op, uint32_t i) {
    if (i == DEPTH) {
        if (verify_lol(op)) {
            pool.push_back(op);
        }
    } else {
        for (int32_t action = 3; action >= 0; action--) {
            op[i] = static_cast<Action>(action);
            generate(op, i + 1);
        }
    }
}

std::vector<Operation> OperationsGenerator::get() {
    Operation op;

    //generate(op, 0);

    // read pool
    {
        //std::ifstream input("Tmp/actions" + std::to_string(get_unique_id()) + ".txt");
        std::stringstream input(
                //call(0): 2507, 12.8034s
                //call(1): 4158, 28.504s
                //call(2): 4983, 35.0189s
                //call(3): 4871, 41.6751s
                //call(4): 3980, 63.2765s
                //call(5): 3183, 99.1406s
                //total: 23682
                "16\nFWW CFW FFW FFF FCF RFC RFF FWF FRF WFW WWF RWF CWF WFF CCF CFF"// 4983

                //call(0): 2527, 12.9228s
                //call(1): 4160, 24.5252s
                //call(2): 4960, 34.0016s
                //call(3): 4885, 47.2288s
                //call(4): 3990, 64.5576s
                //call(5): 3115, 96.0962s
                //total: 23637
                //"16\nFWW CFW FFW FFF FCF RFC RFF FWF FRF RWF CWF CCF CFF WFW WFF WWF" // 4960

                //"8\nFFR RFW CCF FFC FWW FRW CFC WFW" // 2943
                //"9\nFWW CFW RFC FRF WFW WWF RWF CWF CCF" // 3438

                //"4\n FWW CFW RFW CCF" // 863
        );
        uint32_t num;
        input >> num;
        for (uint32_t i = 0; i < num; i++) {
            std::string line;
            input >> line;
            ASSERT(line.size() == op.size(), "does not match sizes: >" + line + "<, " + std::to_string(op.size()));
            for (uint32_t j = 0; j < op.size(); j++) {
                if (line[j] == 'W') {
                    op[j] = Action::W;
                } else if (line[j] == 'F') {
                    op[j] = Action::FW;
                } else if (line[j] == 'R') {
                    op[j] = Action::CR;
                } else if (line[j] == 'C') {
                    op[j] = Action::CCR;
                } else {
                    ASSERT(false, "failed");
                }
            }
            pool.push_back(op);
        }
    }

    //std::reverse(pool.begin(), pool.end());

    // add WWW
    {
        for (uint32_t i = 0; i < op.size(); i++) {
            op[i] = Action::W;
        }
        auto it = std::find(pool.begin(), pool.end(), op);
        if (it != pool.end()) {
            pool.erase(it);
        }
        pool.insert(pool.begin(), op);
    }

    std::vector<Operation> result = pool;

    /*std::set<std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>>> visited;
    for (auto operation: pool) {
        std::array<std::pair<uint32_t, uint32_t>, DEPTH> positions{};
        Position p;
        for (uint32_t d = 0; d < DEPTH; d++) {
            p = p.simulate_action(operation[d]);
            positions[d] = {p.get_x(), p.get_y()};
        }
        std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>> kek = {p.get_dir(), positions};
        if (!visited.count(kek)) {
            visited.insert(kek);
            result.push_back(operation);
        }
    }*/

#ifdef ENABLE_PRINT_LOG
    Printer() << "Operation: " << result.size() << '\n';
    for (auto operation: result) {
        for (uint32_t d = 0; d < DEPTH; d++) {
            char c = '#';
            if (operation[d] == Action::FW) {
                c = 'F';
            } else if (operation[d] == Action::CR) {
                c = 'R';
            } else if (operation[d] == Action::CCR) {
                c = 'C';
            } else if (operation[d] == Action::W) {
                c = 'W';
            } else {
                ASSERT(false, "failed");
            }
            Printer() << c;
        }
        Printer() << '\n';
    }
#endif
    return result;
}

std::vector<Operation> &get_operations() {
    static std::vector<Operation> operations;
    return operations;
}
