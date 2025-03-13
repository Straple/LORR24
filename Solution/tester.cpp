#include <nlohmann/json.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>
#include <Objects/Basic/time.hpp>

#include <fstream>

using json = nlohmann::json;

std::ofstream table_output("table.csv");

// (throughput, milliseconds per steps)
std::pair<double, uint32_t> call(const std::string &test, int steps_num, uint32_t test_id) {
    std::cout << "call(" + std::to_string(test_id) + "): " << std::flush;
    ETimer timer;

    // -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 10000 -t 200000 -p 100000000
    //std::system("mkdir Tmp");
    int ret_code = std::system(
            ("./cmake-build-release-wsl/lifelong -i " + test + " -o Tmp/test" + std::to_string(test_id) +
             ".json -s " + std::to_string(steps_num) + " -t 130 -p 1000000000 -u " + std::to_string(test_id) +
             " > Tmp/log" + std::to_string(test_id) + ".txt")
                    .c_str());

    ASSERT(ret_code == 0, "invalid ret code");

    json data;
    std::ifstream input("Tmp/test" + std::to_string(test_id) + ".json");
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

        std::cout << task_finished << ", " << throughput << ", " << avg_step_time << ", " << timer;
        table_output << agents_num << "," << steps_num << "," << task_finished << "," << throughput << "," << avg_step_time << "," << timer.get_ms() << std::endl;

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

        {"example_problems/random.domain/random_32_32_20_100.json", 1000},
        {"example_problems/random.domain/random_32_32_20_200.json", 1000},
        {"example_problems/random.domain/random_32_32_20_300.json", 1000},
        {"example_problems/random.domain/random_32_32_20_400.json", 1000},
        {"example_problems/random.domain/random_32_32_20_500.json", 1000},
        {"example_problems/random.domain/random_32_32_20_600.json", 1000},
        {"example_problems/random.domain/random_32_32_20_700.json", 1000},
        {"example_problems/random.domain/random_32_32_20_800.json", 1000},
};

int main() {
    table_output << "agents num,steps num,num task finished,throughput,avg step time,total time\n";
    for (uint32_t i = 0; i < tests.size(); i++) {
        call(std::get<0>(tests[i]), std::get<1>(tests[i]), i);
    }
}
