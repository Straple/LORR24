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
    //Operation op;
    //generate(op, 0);

    // read pool
    {
        //std::ifstream input("Tmp/actions" + std::to_string(get_unique_id()) + ".txt");
        std::stringstream input(
                //"17 WWW CWF RWF WWF WFF FWF WFW RRF CFF RFF CFW RFW FCF FRF FWW FFW FFF"

                // default order
                // 6890
                //"45 WWWW RRFW CFWF RFWF FCWF RRRF RWFW RRWF WFWF CFCF RWWF FFWF RRFF RWFF FRWF RFRF CWFW RWCF FWWF CFWW WFFW CWFF WFRF WWFW WWFF WFCF RFWW CFFW WFWW RFFF FWFF FWFW CFFF FWWW RFFW FCFW FRFF FFRF FCFF FFWW FRFW FFCF FFFW WFFF FFFF"

                // 7069
                //"45 WWWW CFWF RFWF FCWF FRWF WFWF RRFW CWWF FWWF RRWF RWFW CFCF RWWF RRFF RWFF FFWF CWFF RFRF CWFW RWCF CFWW FWFF WFFW WFRF WFCF RFWW WFWW WWFF WWFW CFFW FWFW FWWW RFFF CFFF RFFW FFWW FCFW FFRF FFCF FRFF FRFW FCFF WFFF FFFW FFFF"

                // this good
                // 7095
                //"45 WWWW CFWF RFWF FCWF FRWF RRFW CWWF RRWF RWFW CFCF RWWF RRFF RWFF CWFF RFRF CWFW RWCF CFWW WFRF WFCF RFWW CFFW RFFF CFFF RFFW FCFW FFRF FFCF FRFF FRFW FCFF WWFW WWFF WFWW WFWF WFFW WFFF FWWF FFWF FWFW FWFF FWWW FFWW FFFW FFFF"

                // 7209
                //"45 WWWW RRWF WWWF RRFW CWWF RWWF RWFW RRFF CWFW RFWW CFWW RWFF CWFF RFRF WWFW CFCF RFWF CFWF WFWW WWFF WFRF WFCF CFFW RFFW FRWF FCWF FWWW WFWF RFFF CFFF FRFW FCFW WFFW FWWF FCFF FRFF WFFF FWFW FWFF FFWW FFCF FFRF FFWF FFFW FFFF"

                // 7239
                "45 WWWW WWWF RRWF RRFW CWWF RWWF RWFW RRFF CWFW RFWW CFWW RWFF CWFF RFRF CFCF RFWF CFWF WFWW WWFW WWFF WFRF WFCF CFFW RFFW FRWF FCWF FWWW WFWF RFFF CFFF FRFW FCFW WFFW FWWF FCFF FRFF WFFF FWFW FWFF FFWW FFCF FFRF FFWF FFFW FFFF"

                //"121 WWWWW WCCFF WRFWF WWRFF WCFWF CCFWF WRFFF WWCFF WCFCF WRFRF WCFFF WFWFF WFWCF CFWCF RFWRF WFWRF RFWFF CFWFF WFFWF WFWWF FWRFF WWFWF FWCFF WWWFF CCFFF WFFCF WWFFF WCFFW WRFFW CFWWF RFFWF RFWWF CFFWF FFWCF FFWRF WWCCF FWFWF WFFRF WWFCF WWFRF CWFRF FWWCF FWWFF WFWFW RWFCF FWWRF FCFWF WFRFF CFCFF FRFWF WFCFF RFFRF WCCFW RFRFF WWFFW RFWFW CFFCF WWWCF CFWFW WWWRF FWFCF FFWWF FWWWF FWCFW CCFCF CCFFW WWCFW FWFRF FWRFW WWRFW WFFFW WFFFF WFCFW WFRFW WFFWW FWWFW WRFWW WCFWW RRFRF FCFCF FFWFF FWFFF FRFRF WWWWF FWFFW WWWFW CFCFW WWFWW FFFWF FFWFW RFRFW FWFWW RFFFF RFFFW CFFFF CFFFW WFWWW RFWCF CFWRF FFCFF FFRFF FRFFF FFFCF RFFWW FCFFF CCFWW FFFRF FCFFW CFFWW FFCFW FRFFW FFRFW FFFWW RFWWW CFWWW FWWWW FFWWW FCFWW FRFWW FFFFW FFFFF"
                //"121 WWWWW WWCCF WCCFW CCFWW CCFCF WWWRF WWWCF RRFRF WCCFF WWCFW WWRFW WWWWF CCFWF WCFWW CWFRF WRFWW RWFCF WRFRF CCFFW WWRFF WCFCF WWWFW WWCFF RFWWW CFWWW CFWCF CFWRF RFWCF WCFWF WRFWF RFWRF CCFFF WWWFF WWFWW WWFCF WWFRF RFWWF RFRFW CFCFW CFWWF WRFFW WCFFW WFWWW WWFWF WFWCF CFWFW WFWRF RFWFW CFCFF RFRFF WCFFF WRFFF FWWWW WWFFW WFCFW WFRFW WFWWF CFFWW FWWRF FWWCF RFFWW RFWFF CFWFF RFFRF CFFCF FWRFW WFRFF FWWWF WFCFF WWFFF WFWFW FWCFW RFFWF CFFWF FRFWW FCFWW FWWFW FCFCF WFFWW WFWFF FRFRF FWRFF FWCFF WFFCF RFFFW CFFFW WFFRF FRFWF FCFWF FWWFF WFFWF FWFWW FWFCF CFFFF RFFFF FWFRF FRFFW FCFFW FFWWW FWFWF WFFFW FFWRF FFWCF FRFFF FCFFF FFWWF FWFFW WFFFF FFRFW FFCFW FFWFW FFRFF FFCFF FWFFF FFFWW FFWFF FFFRF FFFCF FFFWF FFFFW FFFFF"

                //"121 WWWWW WRFWF WFWRF WFWCF WCFWF CWFRF WWRFF WRFFW RFWRF RWFCF WRFRF WCCFF CFWCF WCFFW WCFCF WWCFF FWRFW FWRFF RFWWF RFWFW FWCFF CCFWF RFWFF FWCFW WRFFF FWWRF WWRFW FWWCF WFFCF FFWRF WWCFW WRFWW CFWFW FFWCF CFWWF WCFFF CFWFF WFFRF FCFWF WCFWW FRFWF WWWRF WCCFW RFFRF WWWCF RFFWF WWCCF WFRFW WFCFW WWFWF WFWWF WFWFW WWFCF RFWCF WWFRF CFFWF CFWRF WWFFW CFFCF RRFRF WWFWW CCFFW CCFCF FWFCF FWFRF RFRFF WWWFW WFRFF CFCFF WFCFF WWWFF WFFWW FWWFW WFWWW RFRFW FWFWW WWWWF FRFRF WFFWF CCFFF CFCFW FWWWF WFWFF RFFWW FWFWF CCFWW FCFCF FFRFW FFCFW FRFFW RFFFW CFFWW FWFFW CFFFW FCFFW RFWWW FRFWW FCFWW FFWFW WFFFW CFWWW FWWFF WWFFF FFWWF FFFCF FFFRF FFRFF FFCFF RFFFF FCFFF FRFFF CFFFF FFWWW FWWWW FFFWW FFWFF FWFFF FFFWF WFFFF FFFFW FFFFF"
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
    }

    /*auto kek = [&](Operation op) {
        double s = 0;
        for (uint32_t d = 0; d < op.size(); d++) {
            s += (op[d] == Action::FW) * (op.size() - d) * 3;
            s -= (op[d] == Action::CR) * (op.size() - d);
            s -= (op[d] == Action::CCR) * (op.size() - d);
        }
        return s;
    };
    std::sort(pool.begin(), pool.end(), [&](auto lhs, auto rhs) {
        return kek(lhs) < kek(rhs);
    });*/

    //std::reverse(pool.begin(), pool.end());

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

    std::vector<Operation> result = pool;

    /*std::set<std::tuple<uint32_t, std::array<std::pair<uint32_t, uint32_t>, DEPTH>>> visited;
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
    }*/

    PRINT(
            Printer() << "Operations:\n"
                      << result.size() << ' ';
            for (auto operation
                 : result) {
                Printer() << operation << ' ';
            } Printer()
            << '\n';);
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
