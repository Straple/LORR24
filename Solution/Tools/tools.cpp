#include <Tools/tools.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/map.hpp>

#include "../../inc/nlohmann/json.hpp"
using json = nlohmann::json;

void build_meta_info(const std::string &filename) {
    std::ofstream output("output.txt");
    output << get_map().get_rows() << ' ' << get_map().get_cols() << std::endl;
    json data;
    std::ifstream input(filename);
    try {
        data = json::parse(input);
    } catch (const json::parse_error &error) {
        std::cerr << "Failed to load " << filename << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    // [pos][dir][action]
    std::vector<std::array<std::array<uint32_t, 3>, 4>> dp(get_map().get_size());

    std::vector<Position> starts;

    for (auto start: data["start"]) {
        std::string dir = start[2];
        uint32_t dir_val = dir == "E" ? 0 : (dir == "S" ? 1 : (dir == "W" ? 2 : (dir == "N" ? 3 : (ASSERT(false, "failed"), -1))));
        Position pos(start[0], start[1], dir_val);
        starts.push_back(pos);
    }

    uint32_t id = 0;
    for (const auto &path_line: data["actualPaths"]) {
        std::string path = path_line;
        Position pos = starts[id];
        for (int i = 0; i < path.size(); i += 2) {
            char c = path[i];
            Action act = c == 'F' ? Action::FW : (c == 'R' ? Action::CR : (c == 'C' ? Action::CCR : (c == 'W' ? Action::W : (ASSERT(false, "invalid action"), Action::NA))));

            dp[pos.get_pos()][pos.get_dir()][static_cast<uint32_t>(act)]++;

            pos = pos.simulate_action(act);
        }
        id++;
    }

    for (uint32_t dir = 0; dir < 4; dir++) {
        for (uint32_t act = 0; act < 4; act++) {
            for (uint32_t pos = 0; pos < get_map().get_size(); pos++) {
                if (pos != 0) {
                    output << ' ';
                }
                output << dp[pos][dir][act];
            }
            output << '\n';
        }
        output << '\n';
    }
}
