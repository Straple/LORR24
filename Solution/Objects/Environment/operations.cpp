#include <Objects/Environment/operations.hpp>

#include <Objects/Basic/assert.hpp>

#include <fstream>

bool verify_lol(const Operation &op) {
    return op[0] != Action::W;

    for (uint32_t i = 0; i + 1 < op.size(); i++) {
        if (op[i] == Action::W && op[i + 1] != Action::W) {
            return false;
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
        for (int32_t action = 3; action >= 0; action--) {
            op[i] = static_cast<Action>(action);
            generate(op, i + 1);
        }
    }
}

std::vector<Operation> OperationsGenerator::get() {
    Operation op;

    // read pool
    {
        //std::ifstream input("Tmp/actions" + std::to_string(get_unique_id()) + ".txt");
        std::stringstream input(
                "16\nFWW CFW FFW FFF FCF RFC RFF FWF FRF WFW WWF RWF CWF WFF CCF CFF"
                //"12\nFRR CFC RFC CCR FRF FRW RCC WFR FRC FCC RRR WWC"
                //"8\nFFR RFW CCF FFC FWW FRW CFC WFW" // 3110

        );
        // score: 3017
        // "16\nFWW CFW FFW FFF FCF RFC RFF FWF FRF WFW WWF RWF CWF WFF CCF CFF"
        // score: 2805
        //"12\nFRR CFC RFC CCR FRF FRW RCC WFR FRC FCC RRR WWC"
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

    // add WWW
    {
        for (uint32_t i = 0; i < op.size(); i++) {
            op[i] = Action::W;
        }
        pool.insert(pool.begin(), op);
    }

    std::vector<Operation> result = pool;

    /*std::set<std::array<std::pair<uint32_t, uint32_t>, DEPTH>> visited;
    for (auto operation: pool) {
        std::array<std::pair<uint32_t, uint32_t>, DEPTH> positions{};
        Position p;
        std::set<std::pair<uint32_t, uint32_t>> S;
        for (uint32_t d = 0; d < DEPTH; d++) {
            p = p.simulate_action(operation[d]);
            positions[d] = {p.get_x(), p.get_y()};
            S.insert(positions[d]);
        }
        if (S.size() > 2) {
            //continue;
        }
        std::array<std::pair<uint32_t, uint32_t>, DEPTH> kek = positions;
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
