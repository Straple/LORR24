#include <Objects/Environment/operations.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>

#include <fstream>

bool verify_lol(const Operation &op) {
    for (int32_t i = op.size() - 1; i >= 0; i--) {
        if (op[i] == Action::FW) {
            break;
        }
        if (op[i] == Action::CCR || op[i] == Action::CR) {
            return false;
        }
    }
    for (int i = 0; i < op.size(); i++) {
        for (int j = i + 1; j < op.size(); j++) {
            if (op[i] == Action::CR && op[j] == Action::CCR) {
                return false;
            }
            if (op[i] == Action::CCR && op[j] == Action::CR) {
                return false;
            }
            if (op[j] == Action::FW) {
                break;
            }
        }
    }
    return true;
}

void OperationsGenerator::generate(Operation &op, uint32_t i) {
    if (i == DEPTH) {
        if (verify_lol(op)) {
            pool.push_back(op);
        }
    } else {
        for (int32_t action = 0; action < 4; action++) {
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
                "16 FFF FFW FWF FWW WFF WFW WWF FCF FRF RFF CFF RFW CFW RRF RWF CWF"
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

    auto kek = [&](Operation op){
        int cnt = 0;
        for(int i = 0; i < op.size(); i++){
            cnt += op[i] == Action::FW;
        }
        return cnt;
    };

    //call(0): 2391, 15.1243s
    //call(1): 4116, 25.4063s
    //call(2): 5165, 45.2252s
    //call(3): 5766, 58.3878s
    //call(4): 4479, 100.457s
    //call(5): 4025, 186.126s
    //total: 25942
    //std::stable_sort(pool.begin(), pool.end(), [&](auto lhs, auto rhs){
    //    return kek(lhs) < kek(rhs);
    //});

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
        std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>> kek = {0, positions};
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
