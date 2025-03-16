#include <nlohmann/json.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>
#include <Objects/Basic/time.hpp>

#include <fstream>

using json = nlohmann::json;

std::ofstream table_output;

// (throughput, milliseconds per steps)
std::pair<double, uint32_t> call(const std::string &test, int steps_num, const std::string &plan_algo, uint32_t test_id) {
    std::cout << "call(" + std::to_string(test_id) + ", " << plan_algo << "): " << std::flush;
    ETimer timer;

    {
        int ret_code = std::system(
                (std::string("./bin/lifelong") +                                       //
                 " -i " + test +                                                       //
                 " -o Tmp/" + plan_algo + "/test" + std::to_string(test_id) + ".json" +//
                 " -s " + std::to_string(steps_num) +                                  //
                 " -t 1000 " +                                                         //
                 " -p 1000000000" +                                                    //
                 " -u " + std::to_string(test_id) +                                    //
                 " --planner_algo " + plan_algo +                                      //
                 " > Tmp/" + plan_algo + "/output" + std::to_string(test_id) + ".txt"  //
                 )
                        .c_str());

        ASSERT(ret_code == 0, "invalid ret code");
    }

    json data;
    std::ifstream input("Tmp/" + plan_algo + "/test" + std::to_string(test_id) + ".json");
    double throughput = 0;
    uint32_t avg_step_time = 0;
    uint32_t task_finished = 0;
    uint32_t agents_num = 0;
    try {
        data = json::parse(input);

        task_finished = data["numTaskFinished"];
        throughput = static_cast<double>(data["numTaskFinished"]) / steps_num;
        std::vector<double> times = data["plannerTimes"];
        avg_step_time = static_cast<uint32_t>(std::accumulate(times.begin(), times.end(), 0.0) * 1000 / steps_num);
        agents_num = data["teamSize"];

        auto kek = [&](double x) {
            std::stringstream ss;
            ss << x;
            std::string str = ss.str();
            for (char &c: str) {
                if (c == '.') {
                    c = ',';
                }
            }
            return str;
        };

        std::cout << task_finished << ", " << timer;
        table_output << test_id << ";" << plan_algo << ";" << agents_num << ";" << steps_num << ";" << task_finished << ";" << kek(throughput) << ";" << avg_step_time << ";" << timer.get_ms() << std::endl;

        if (data["numEntryTimeouts"] != 0) {
            std::cout << " ENTRY TIMEOUT";
        }
        if (data["numPlannerErrors"] != 0) {
            std::cout << " PLANNER ERROR";
        }
        if (data["numScheduleErrors"] != 0) {
            std::cout << "SCHEDULER ERROR";
        }

        std::cout << std::endl;

    } catch (const json::parse_error &error) {
        std::cerr << "Message: " << error.what() << std::endl;
    }

    // build usage plots
    {
        int ret_code = std::system(("python3 Solution/Python/build_usage_plot.py Tmp/" + plan_algo + "/usage" + std::to_string(test_id) + ".txt Tmp/" + plan_algo + "/usage_plot" + std::to_string(test_id) + "_one.pdf Tmp/" + plan_algo + "/usage_plot" + std::to_string(test_id) + "_all.pdf").c_str());
        ASSERT(ret_code == 0, "invalid ret code");
    }

    return {throughput, avg_step_time};
}

std::vector<std::tuple<std::string, int>> tests = {
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

        /*{"example_problems/random.domain/random_32_32_20_100.json", 1000},
        {"example_problems/random.domain/random_32_32_20_200.json", 1000},
        {"example_problems/random.domain/random_32_32_20_300.json", 1000},
        {"example_problems/random.domain/random_32_32_20_400.json", 1000},
        {"example_problems/random.domain/random_32_32_20_500.json", 1000},
        {"example_problems/random.domain/random_32_32_20_600.json", 1000},
        {"example_problems/random.domain/random_32_32_20_700.json", 1000},
        {"example_problems/random.domain/random_32_32_20_800.json", 1000},*/

        {"example_problems/warehouse.domain/warehouse_large_1000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_2000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_3000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_4000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_5000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_6000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_7000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_8000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_9000.json", 5000},
        {"example_problems/warehouse.domain/warehouse_large_10000.json", 5000},

        /*{"example_problems/game.domain/brc202d_1000.json", 5000},
        {"example_problems/game.domain/brc202d_2000.json", 5000},
        {"example_problems/game.domain/brc202d_3000.json", 5000},
        {"example_problems/game.domain/brc202d_4000.json", 5000},
        {"example_problems/game.domain/brc202d_5000.json", 5000},
        {"example_problems/game.domain/brc202d_6000.json", 5000},
        {"example_problems/game.domain/brc202d_7000.json", 5000},
        {"example_problems/game.domain/brc202d_8000.json", 5000},
        {"example_problems/game.domain/brc202d_9000.json", 5000},
        {"example_problems/game.domain/brc202d_10000.json", 5000},*/
};

int main() {

    std::vector<std::string> plan_algos = {
            //"pibt",
            //"pibt_tf",
            "epibt",
            "epibt_lns",
            "pepibt_lns",
            "wppl",
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
        table_output = std::ofstream("Tmp/" + plan_algo + "/metrics.csv");
        table_output << "id;planner algo;agents num;steps num;num task finished;throughput;avg step time;total time\n";
        uint32_t counter = 0;
        for (auto [test, steps_num]: tests) {
            call(test, steps_num, plan_algo, counter);
            counter++;
        }
    }
}
