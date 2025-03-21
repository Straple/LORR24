#include <nlohmann/json.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>

#include <fstream>

using json = nlohmann::json;

void call(const std::string &test, int steps_num, const std::string &plan_algo, uint32_t test_id) {
    std::cout << "call(" + std::to_string(test_id) + ", " << plan_algo << "): " << std::flush;
    ETimer timer;

    int ret_code = std::system(
            (std::string("./bin/lifelong") +                                       //
             " -i " + test +                                                       //
             " -o Tmp/" + plan_algo + "/test" + std::to_string(test_id) + ".json" +//
             " -s " + std::to_string(steps_num) +                                  //
             " -t 2000 " +                                                         //
             " -p 1000000000" +                                                    //
             " -u " + std::to_string(test_id) +                                    //
             " --planner_algo " + plan_algo +                                      //
             " > Tmp/" + plan_algo + "/output" + std::to_string(test_id) + ".txt"  //
             )
                    .c_str());

    //ASSERT(ret_code == 0, "invalid ret code");

    std::cout << timer << std::endl;
    if (ret_code != 0) {
        call(test, steps_num, plan_algo, test_id);
    }
}

std::vector<std::tuple<std::string, int, bool>> tests = {
        /*{"Data2023/city.domain/MR23-I-01.json", 1500},
        {"Data2023/city.domain/MR23-I-02.json", 3500},
        {"Data2023/random.domain/MR23-I-03.json", 500},
        {"Data2023/random.domain/MR23-I-04.json", 500},
        {"Data2023/random.domain/MR23-I-05.json", 1000},
        {"Data2023/warehouse.domain/MR23-I-06.json", 5000},
        {"Data2023/random.domain/MR23-I-07.json", 1000},
        {"Data2023/random.domain/MR23-I-08.json", 2000},
        {"Data2023/game.domain/MR23-I-09.json", 5000},
        {"Data2023/warehouse.domain/MR23-I-10.json", 5000},*/

        /*{"example_problems/random.domain/random_32_32_20_100.json", 1000, false},
        {"example_problems/random.domain/random_32_32_20_200.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_300.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_400.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_500.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_600.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_700.json", 1000, true},
        {"example_problems/random.domain/random_32_32_20_800.json", 1000, true},*/

        {"example_problems/warehouse.domain/warehouse_large_1000.json", 5000, false},
        {"example_problems/warehouse.domain/warehouse_large_2000.json", 5000, false},
        {"example_problems/warehouse.domain/warehouse_large_3000.json", 5000, false},
        {"example_problems/warehouse.domain/warehouse_large_4000.json", 5000, false},
        {"example_problems/warehouse.domain/warehouse_large_5000.json", 5000, false},
        {"example_problems/warehouse.domain/warehouse_large_6000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_7000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_8000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_9000.json", 5000, true},
        {"example_problems/warehouse.domain/warehouse_large_10000.json", 5000, true},

        /*{"example_problems/game.domain/brc202d_1000.json", 5000, false},
        {"example_problems/game.domain/brc202d_2000.json", 5000, false},
        {"example_problems/game.domain/brc202d_3000.json", 5000, false},
        {"example_problems/game.domain/brc202d_4000.json", 5000, false},
        {"example_problems/game.domain/brc202d_5000.json", 5000, true},
        {"example_problems/game.domain/brc202d_6000.json", 5000, true},
        {"example_problems/game.domain/brc202d_7000.json", 5000, true},
        {"example_problems/game.domain/brc202d_8000.json", 5000, true},
        {"example_problems/game.domain/brc202d_9000.json", 5000, true},
        {"example_problems/game.domain/brc202d_10000.json", 5000, true},*/
};

// GAME, EPIBT_LNS нужно пересчитать: operations bug, но вообще можно его не включать просто

// на том сервере WPPL и pibt_tf до конца посчитаны

int main() {

    std::vector<std::string> plan_algos = {
            //"pibt",
            //"epibt",
            //"epibt_lns",
            //"pepibt_lns",
            //"wppl",
            "pibt_tf",
    };

    if (!std::filesystem::exists("Tmp")) {
        std::filesystem::create_directories("Tmp");
    }

    for (const auto &plan_algo: plan_algos) {
        if (!std::filesystem::exists("Tmp/" + plan_algo)) {
            std::filesystem::create_directories("Tmp/" + plan_algo);
        }
    }

    for (const auto &plan_algo: plan_algos) {
        uint32_t counter = 0;
        for (auto [test, steps_num, need_to_call]: tests) {
            if (need_to_call) {
                call(test, steps_num, plan_algo, counter);
            }
            counter++;
        }
    }
}
