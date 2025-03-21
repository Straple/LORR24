#include <nlohmann/json.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/time.hpp>

#include <fstream>

using json = nlohmann::json;

std::ofstream table_output;

std::ofstream total_table_output;

void call(const std::string &map_name, const std::string &plan_algo, uint32_t test_id) {
    json data;
    std::ifstream input("Data_" + map_name + "/" + plan_algo + "/test" + std::to_string(test_id) + ".json");
    if (!input) {
        std::cout << "unable ";
        return;
    }
    double throughput = 0;
    uint32_t avg_step_time = 0;
    uint32_t task_finished = 0;
    uint32_t agents_num = 0;
    try {
        data = json::parse(input);

        uint32_t steps_num = data["makespan"];
        task_finished = data["numTaskFinished"];
        throughput = static_cast<double>(data["numTaskFinished"]) / steps_num;
        std::vector<double> times = data["plannerTimes"];
        avg_step_time = static_cast<uint32_t>(std::accumulate(times.begin(), times.end(), 0.0) * 1000 / steps_num);
        agents_num = data["teamSize"];

        double CI_left = 0;
        double CI_right = 0;
        {
            double avgx = avg_step_time;
            double sigma = std::sqrt(std::accumulate(times.begin(), times.end(), 0, [&](double sum, double x) {
                                         return sum + (x - avgx) * (x - avgx);
                                     }) /
                                     (times.size() - 1));

            double z = 1.96;
            double sem = sigma / std::sqrt(times.size());
            CI_left = avgx - z * sem;
            CI_right = avgx + z * sem;
        }

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

        table_output << test_id << ";" << plan_algo << ";" << agents_num << ";" << steps_num << ";" << task_finished << ";" << kek(throughput) << ";" << avg_step_time << ";" << CI_left << ";" << CI_right << std::endl;
        total_table_output << test_id << ";" << plan_algo << ";" << agents_num << ";" << steps_num << ";" << task_finished << ";" << kek(throughput) << ";" << avg_step_time << ";" << CI_left << ";" << CI_right << std::endl;

        if (data["numEntryTimeouts"] != 0) {
            std::cerr << "\nENTRY TIMEOUT\n";
        }
        if (data["numPlannerErrors"] != 0) {
            std::cerr << "\nPLANNER ERROR\n";
        }
        if (data["numScheduleErrors"] != 0) {
            std::cerr << "\nSCHEDULER ERROR\n";
        }

    } catch (const json::parse_error &error) {
        std::cerr << "Failed at: "
                  << "Data_" + map_name + "/" + plan_algo + "/test" + std::to_string(test_id) + ".json" << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
    }

    // build usage plots
    {
        //int ret_code = std::system(("python3 Solution/Python/build_usage_plot.py Data_" + map_name + "/" + plan_algo + "/usage" + std::to_string(test_id) + ".txt Data_" + map_name + "/" + plan_algo + "/usage_plot" + std::to_string(test_id) + "_one.pdf Data_" + map_name + "/" + plan_algo + "/usage_plot" + std::to_string(test_id) + "_all.pdf").c_str());
        //ASSERT(ret_code == 0, "invalid ret code");
    }
}

int main() {

    std::vector<std::string> maps_name = {
            "random",
            "warehouse",
            "game",
    };

    std::vector<std::string> plan_algos = {
            "pibt",
            "pibt_tf",
            "epibt",
            "epibt_lns",
            "pepibt_lns",
            "wppl",
    };

    for (const auto &map_name: maps_name) {
        total_table_output = std::ofstream("Data_" + map_name + "/total_metrics.csv");
        total_table_output << "id;planner algo;agents num;steps num;num task finished;throughput;avg step time;CI left;CI right\n";
        for (const auto &plan_algo: plan_algos) {
            table_output = std::ofstream("Data_" + map_name + "/" + plan_algo + "/metrics.csv");
            table_output << "id;planner algo;agents num;steps num;num task finished;throughput;avg step time;CI left;CI right\n";
            for (uint32_t test_id = 0; test_id < 10; test_id++) {
                ETimer timer;
                std::cout << "call(" << map_name << ' ' << plan_algo << ' ' << test_id << "): " << std::flush;
                call(map_name, plan_algo, test_id);
                std::cout << timer << std::endl;
            }
        }
    }
}
