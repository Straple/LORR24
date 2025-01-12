#include <fstream>

#include <Objects/Environment/graph_guidance.hpp>
#include <Objects/GraphGuidanceBuilder/graph_guidance_solver.hpp>

int main() {
    Printer().get() = std::ofstream("gg_log");

    /*{
        std::ofstream output("best_gg");
        GraphGuidance gg(32, 32);
        for (uint32_t pos = 0; pos < 32 * 32 + 1; pos++) {
            for (uint32_t dir = 0; dir < 4; dir++) {
                for (uint32_t action = 0; action < 4; action++) {
                    gg.set(pos, dir, action, 500);
                }
            }
        }
        output << gg;
        output.flush();
        return 0;
    }*/
    //std::string params = "-i ./example_problems/random.domain/random_32_32_20_300.json -o test.json -s 2000 -t 100000000 -p 100000000";
    /*std::string params;
    for (int i = 1; i < argc; i++) {
        params += argv[i];
        params += ' ';
    }
    std::cout << "params: " << params << '\n';*/

    GraphGuidance gg;
    {
        std::ifstream input("best_gg");
        input >> gg;
    }

    int dhm_power = 500;
    {
        std::ifstream input("best_args");
        input >> dhm_power;
    }

    GraphGuidanceSolver ggs(gg, dhm_power);
    ggs.solve();
}
