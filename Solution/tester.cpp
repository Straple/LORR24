#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>

using json = nlohmann::json;

uint32_t call(const std::string &test, uint32_t test_id) {
    std::cout << "call(" +std::to_string(test_id) + "): " << std::flush;
    Timer timer;

    // -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 10000 -t 200000 -p 100000000
    int ret_code = std::system(
            ("./cmake-build-release-wsl/lifelong -i example_problems/" + test + " -o test" + std::to_string(test_id) + ".json -s 1000 -t 300 -p 100000000 > log" + std::to_string(test_id) + ".txt").c_str());

    ASSERT(ret_code == 0, "invalid ret code");

    json data;
    std::ifstream input("test" + std::to_string(test_id) + ".json");
    try {
        data = json::parse(input);
    } catch (const json::parse_error &error) {
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    uint32_t value = data["numTaskFinished"];
    std::cout << value << ", " << timer << std::endl;
    return value;
}

std::vector<std::string> tests = {
        "random.domain/random_32_32_20_100.json",
        "random.domain/random_32_32_20_200.json",
        "random.domain/random_32_32_20_300.json",
        "random.domain/random_32_32_20_400.json",
        "random.domain/random_32_32_20_500.json",
        "random.domain/random_32_32_20_600.json",
};

uint32_t call() {
    uint32_t res = 0;
    for (uint32_t i = 0; i < tests.size(); i++) {
        res += call(tests[i], i);
    }
    return res;
}

int main() {
    std::cout << call() << std::endl;
}

// PIBT2:

/*
10k:
call(0): 22782, 12.9289s
call(1): 31208, 23.0029s
call(2): 23005, 76.5596s
call(3): 17444, 178.088s
call(4): 16060, 290.57s
call(5): 13260, 465.695s
123759
*/

/*
1k:
call(0): 2264, 3.12296s
call(1): 3309, 4.10285s
call(2): 3017, 5.50353s
call(3): 2701, 6.12878s
call(4): 2548, 8.68127s
call(5): 2276, 16.7255s
16115
*/

// PIBT3
/*
1k
call(0): 2222, 2.71428s
call(1): 2662, 4.41944s
call(2): 2212, 7.35723s
call(3): 1835, 11.9417s
call(4): 1553, 23.9391s
call(5): 1355, 30.2249s
11839
*/
