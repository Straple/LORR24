#include <Objects/Environment/operations.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/position.hpp>

#include <fstream>

bool verify_lol(const Operation &op) {
    // не нужно, чтобы на конце операции были повороты
    for (int i = op.size() - 1; i >= 0 && op[i] != Action::FW; i--) {
        if (op[i] == Action::CR || op[i] == Action::CCR) {
            return false;
        }
    }
    for (int i = 0; i < op.size(); i++) {
        for (int j = i + 1; j < op.size() && op[j] != Action::W; j++) {
            if (op[i] == Action::CR && op[j] == Action::CCR) {
                return false;
            }
            if (op[i] == Action::CCR && op[j] == Action::CR) {
                return false;
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
    generate(op, 0);

    // read pool
    /*{
        //std::ifstream input("Tmp/actions" + std::to_string(get_unique_id()) + ".txt");
        std::stringstream input(
                "16 FFF FFW FWW FRF FCF RFW CFW RFF CFF RRF WFW FWF WFF WWF RWF CWF"
        );
        uint32_t num;
        input >> num;
        for (uint32_t i = 0; i < num; i++) {
            std::string line;
            input >> line;
            Operation op;
            ASSERT(line.size() == op.size(), "does not match sizes: >" + line + "<, " + std::to_string(op.size()));
            std::stringstream ss(line);
            ss >> op;
            pool.push_back(op);
        }
    }*/

    // add WWW
    {
        Operation op;
        for (uint32_t i = 0; i < op.size(); i++) {
            op[i] = Action::W;
        }
        auto it = std::find(pool.begin(), pool.end(), op);
        if (it != pool.end()) {
            pool.erase(it);
        }
        pool.insert(pool.begin(), op);
    }

    std::vector<Operation> result;

    std::set<std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>>> visited;
    for (auto operation: pool) {
        std::array<std::pair<uint32_t, uint32_t>, DEPTH> positions{};
        Position p;
        std::set<std::pair<uint32_t, uint32_t>> visited_poses;
        visited_poses.insert({p.get_x(), p.get_y()});
        bool ok = true;
        for (uint32_t d = 0; d < DEPTH; d++) {
            p = p.simulate_action(operation[d]);
            positions[d] = {p.get_x(), p.get_y()};
            if (operation[d] == Action::FW && visited_poses.count(positions[d])) {
                ok = false;
            }
            visited_poses.insert(positions[d]);
        }
        std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>> kek = {p.get_dir(), positions};
        if (!visited.count(kek) && ok) {
            visited.insert(kek);
            result.push_back(operation);
        }
    }

#ifdef ENABLE_PRINT_LOG
    Printer() << "Operation: " << result.size() << '\n';
    for (auto operation: result) {
        Printer() << operation << '\n';
    }
#endif
    //std::exit(100);
    return result;
}

std::vector<Operation> &get_operations() {
    static std::vector<Operation> operations;
    return operations;
}

std::vector<int> &get_operations_weights() {
    static std::vector<int> weights;
    return weights;
}

std::ostream &operator<<(std::ostream &output, const Operation &op) {
    for (auto op: op) {
        if (op == Action::W) {
            output << 'W';
        } else if (op == Action::FW) {
            output << 'F';
        } else if (op == Action::CCR) {
            output << 'C';
        } else if (op == Action::CR) {
            output << 'R';
        } else if (op == Action::NA) {
            output << 'N';
        }
    }
    return output;
}

std::istream &operator>>(std::istream &input, Operation &op) {
    for (auto &op: op) {
        char c;
        input >> c;
        if (c == 'W') {
            op = Action::W;
        } else if (c == 'F') {
            op = Action::FW;
        } else if (c == 'C') {
            op = Action::CCR;
        } else if (c == 'R') {
            op = Action::CR;
        } else if (c == 'N') {
            op = Action::NA;
        } else {
            ASSERT(false, "invalid action");
        }
    }
    return input;
}
