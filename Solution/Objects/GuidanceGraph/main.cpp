#include <fstream>

#include "../guidance_graph.hpp"
#include "guidance_graph_solver.hpp"

int main(int argc, char **argv) {
    /*std::ofstream output("best_gg.txt");
    uint32_t size = ROWS * COLS;
    uint16_t val = 10000;
    get_gg().graph.resize(size);
    for (uint32_t pos = 0; pos < size; pos++) {
        for (uint32_t dir = 0; dir < 4; dir++) {
            for (uint32_t action = 0; action < 3; action++) {
                get_gg().graph[pos][dir][action] = val;
            }
        }
    }
    output << get_gg();
    return 0;*/

    std::string params;
    for (int i = 1; i < argc; i++) {
        params += argv[i];
        params += ' ';
    }
    std::cout << "params: " << params << '\n';

    GuidanceGraph gg;
    std::ifstream input("best_gg.txt");
    input >> gg;

    GuidanceGraphSolver ggs(gg, params);
    ggs.solve();
}
