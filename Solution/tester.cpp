#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>
#include <Planner/PIBT/pibt2.hpp>

using json = nlohmann::json;

uint32_t call(const std::string &test, uint32_t test_id) {
    std::cout << "call(" +std::to_string(test_id) + "): " << std::flush;
    Timer timer;

    // -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 10000 -t 200000 -p 100000000
    int ret_code = std::system(
            ("./cmake-build-release-wsl/lifelong -i example_problems/" + test + " -o Tmp/test" + std::to_string(test_id) + ".json -s 1000 -t 100000 -p 100000000 > Tmp/log" + std::to_string(test_id) + ".txt").c_str());

    ASSERT(ret_code == 0, "invalid ret code");

    json data;
    std::ifstream input("Tmp/test" + std::to_string(test_id) + ".json");
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
    uint32_t total = call();
    std::cout << "total: " << total << std::endl;
}

/*
MAPFPLANNER
10k
call(0): 21740, 303.916s
call(1): 34562, 299.623s
call(2): 31075, 292.259s
call(3): 26337, 286.827s
call(4): 24899, 280.435s
call(5): 23453, 273.153s
total: 162066

1k
call(0): 2169, 30.7614s
call(1): 3447, 30.0941s
call(2): 3622, 29.5492s
call(3): 3317, 28.8547s
call(4): 2797, 28.4491s
call(5): 2358, 27.6499s
total: 17710
*/

/*
PIBT2

10k
call(0): 22747, 21.7433s
call(1): 32159, 26.0835s
call(2): 28078, 44.2924s
call(3): 24978, 68.7657s
call(4): 22338, 108.811s
call(5): 20170, 147.452s
total: 150470

1k
call(0): 2264, 1.77612s
call(1): 3309, 2.44252s
call(2): 3017, 3.35352s
call(3): 2701, 4.90992s
call(4): 2577, 5.88574s
call(5): 2237, 8.23335s
total: 16105
*/

/*
PIBTS:
call(0): 2409, 30.8636s
call(1): 3974, 30.8771s
call(2): 4073, 31.1857s
call(3): 3624, 31.2429s
call(4): 3212, 31.5072s
call(5): 2592, 31.7745s
total: 19884

call(0): 23963, 306.801s
call(1): 38591, 309.525s
call(2): 33340, 313.238s
call(3): 23042, 315.141s
call(4): 19438, 318.031s
call(5): 18218, 322.4s
total: 156592

500 solver steps
call(0): 2420, 6.16338s
call(1): 3962, 5.70953s
call(2): 4118, 12.6022s
call(3): 3592, 27.3051s
call(4): 3081, 59.3801s
call(5): 2594, 92.1525s
total: 19767

PIBT2 + 500 solver steps
call(0): 2404, 6.25056s
call(1): 3864, 10.0511s
call(2): 4106, 33.6654s
call(3): 3570, 46.5616s
call(4): 3100, 75.3258s
call(5): 2643, 133.274s
total: 19687

PIBT2 + 500 solver steps + SHEDULER_TRICK
call(0): 2448, 14.288s
call(1): 3875, 21.5622s
call(2): 4245, 38.3575s
call(3): 3639, 111.126s
call(4): 3218, 133.659s
call(5): 2794, 208.623s
total: 20219

PIBT2 + 500 solver steps + SHEDULER_TRICK + DHM
call(0): 2452, 35.8154s
call(1): 4121, 39.3456s
call(2): 4828, 43.2985s
call(3): 4416, 58.853s
call(4): 3441, 102.102s
call(5): 2909, 124.727s
total: 22167

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
