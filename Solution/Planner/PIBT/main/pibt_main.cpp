#include "Planner/PIBT/pibt2.hpp"
#include "Objects/Basic/assert.hpp"
#include "Objects/Basic/randomizer.hpp"

//#include "../../../inc/nlohmann/json.hpp"

using json = nlohmann::json;

bool verify(const Operation &op) {
    return true;
}

void write_pool(const std::vector<Operation> &pool, std::ofstream &output) {
    output << pool.size() << '\n';
    for (auto op: pool) {
        for (auto &action: op) {
            if (action == Action::W) {
                output << 'W';
            } else if (action == Action::FW) {
                output << 'F';
            } else if (action == Action::CR) {
                output << 'R';
            } else if (action == Action::CCR) {
                output << 'C';
            } else {
                ASSERT(false, "failed");
            }
        }
        output << ' ';
    }
    output << '\n';
    output.flush();
}

void generate(Operation &op, uint32_t i, std::vector<Operation> &pool) {
    if (i == DEPTH) {
        if (verify(op)) {
            pool.push_back(op);
        }
    } else {
        for (int32_t action = 0; action < 4; action++) {
            op[i] = static_cast<Action>(action);
            generate(op, i + 1, pool);
        }
    }
}

uint32_t call(const std::vector<Operation> &pool) {
    std::cout << "call: " << std::flush;

    {
        std::ofstream output("actions.txt");
        write_pool(pool, output);
    }

    // -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 10000 -t 200000 -p 100000000
    std::system(
            "./lifelong -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 5000 -p 100000000 > log.txt");

    json data;
    std::ifstream input("test.json");
    try {
        data = json::parse(input);
    } catch (const json::parse_error &error) {
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    uint32_t value = data["numTaskFinished"];
    std::cout << value << std::endl;
    return value;
}

int main() {
    std::vector<Operation> pool;
    {
        Operation op;
        generate(op, 0, pool);
    }

    std::cout << pool.size() << std::endl;

    Randomizer rnd;
    std::ofstream ansput("answers.txt");

    uint32_t best = 0;

    while (true) {
        std::vector<Operation> state = pool;
        std::shuffle(state.begin(), state.end(), rnd.generator);
        std::cout << "NEW " << best << std::endl;
        uint32_t ans_score = call(state);

        auto try_to_swap = [&]() {
            uint32_t a = rnd.get(0, state.size() - 1);
            uint32_t b = rnd.get(0, state.size() - 1);
            std::swap(state[a], state[b]);

            uint32_t cur_score = call(state);

            if (cur_score > best) {
                best = cur_score;
                ansput << "SCORE: " << cur_score << std::endl;
                write_pool(state, ansput);
                ansput << std::endl;
            }

            if (cur_score >= ans_score) {
                ans_score = cur_score;
            } else {
                std::swap(state[a], state[b]);
            }
        };

        auto try_to_reverse = [&]() {
            uint32_t a = rnd.get(0, state.size() - 1);
            uint32_t b = rnd.get(0, state.size() - 1);
            if (a > b) {
                std::swap(a, b);
            }
            for (int32_t left = a, right = b; left < right; left++, right--) {
                std::swap(state[left], state[right]);
            }

            uint32_t cur_score = call(state);

            if (cur_score > best) {
                best = cur_score;
                ansput << "SCORE: " << cur_score << std::endl;
                write_pool(state, ansput);
                ansput << std::endl;
            }

            if (cur_score >= ans_score) {
                ans_score = cur_score;
            } else {
                for (int32_t left = a, right = b; left < right; left++, right--) {
                    std::swap(state[left], state[right]);
                }
            }
        };

        for (uint32_t step = 0; step < 100; step++) {
            if (rnd.get_d() < 0.3) {
                try_to_reverse();
            } else {
                try_to_swap();
            }
        }
    }
}

/*C:\Windows\system32\wsl.exe --distribution Ubuntu --exec /bin/bash -c "cd /home/straple/LORR24 && /home/straple/LORR24/pibt_main"
num: 64
Operation: 17
WWW
RRF
CWF
FFF
FCC
RFW
FFR
CFC
RWF
WFC
CRF
FRF
FWF
RFF
CFF
FCF
WFF
64
NEW 0
call: 0
call: 0
call: 0
call: 0
call: 0
call: 0
call: 0
call: 0
call: 0
call: 0
call: 0
call: 2034
call: 2034
call: 2034
call: 2034
call: 2034
call: 2118
call: 2096
call: 2191
call: 2172
call: 2101
call: 2174
call: 2191
call: 2191
call: 2176
call: 2191
call: 2191
call: 2124
call: 2191
call: 1958
call: 2137
call: 2191
call: 2176
call: 2191
call: 1984
call: 1965
call: 2097
call: 2191
call: 2105
call: 2174
call: 2191
call: 2191
call: 2191
call: 2191
call: 2102
call: 2139
call: 2191
call: 1975
call: 2176
call: 2191
call: 2097
call: 2191
call: 1447
call: 1851
call: 2188
call: 2191
call: 2081
call: 2191
call: 2191
call: 2191
call: 2118
call: 2174
call: 2191
call: 2191
call: 2191
call: 2069
call: 2191
call: 2172
call: 2191
call: 1950
call: 2191
call: 2191
call: 2191
call: 2139
call: 2118
call: 2191
call: 2086
call: 2191
call: 2090
call: 2191
call: 2166
call: 1988
call: 2139
call: 2191
call: 2095
call: 2191
call: 45
call: 2191
call: 2191
call: 2191
call: 2125
call: 2172
call: 2191
call: 2191
call: 2172
call: 2191
call: 2191
call: 2191
call: 2187
call: 2133
call: 2191
NEW 2191
call: 0
call: 7
call: 7
call: 7
call: 6
call: 9
call: 9
call: 12
call: 12
call: 12
call: 10
call: 12
call: 12
call: 12
call: 11
call: 12
call: 12
call: 66
call: 66
call: 62
call: 58
call: 0
call: 0
call: 66
call: 66
call: 66
call: 78
call: 82
call: 82
call: 82
call: 82
call: 82
call: 82
call: 2054
call: 2054
call: 1883
call: 2053
call: 2054
call: 2063
call: 2063
call: 2087
call: 2087
call: 2087
call: 2087
call: 2
call: 2087
call: 2091
call: 1940
call: 2091
call: 2089
call: 2091
call: 1647
call: 2086
call: 2091
call: 2091
call: 2091
call: 2091
call: 2091
call: 2024
call: 1904
call: 2091
call: 2112
call: 2127
call: 2127
call: 2007
call: 2127
call: 2114
call: 1754
call: 2127
call: 2127
call: 2127
call: 2127
call: 2074
call: 2158
call: 2158
call: 1974
call: 2158
call: 2158
call: 1951
call: 2114
call: 2158
call: 1993
call: 2158
call: 2158
call: 2158
call: 2112
call: 0
call: 1974
call: 2158
call: 2158
call: 2046
call: 2075
call: 2158
call: 2158
call: 2158
call: 2124
call: 2158
call: 2075
call: 2104
call: 2158
call: 2044
NEW 2191
call: 1924
call: 1924
call: 2004
call: 1379
call: 14
call: 1858
call: 2005
call: 1980
call: 2024
call: 1878
call: 1981
call: 0
call: 2073
call: 2122
call: 1961
call: 2122
call: 2122
call: 832
call: 2124
call: 0
call: 2124
call: 2124
call: 2124
call: 2124
call: 1982
call: 2006
call: 2124
call: 2124
call: 1998
call: 2124
call: 1863
call: 2124
call: 2124
call: 0
call: 2141
call: 2110
call: 2077
call: 2108
call: 2141
call: 1937
call: 1982
call: 2043
call: 2029
call: 2075
call: 2
call: 2141
call: 2141
call: 2141
call: 1961
call: 2141
call: 2141
call: 2
call: 2078
call: 2141
call: 2141
call: 2141
call: 2141
call: 2065
call: 1935
call: 1899
call: 1841
call: 2141
call: 2212
call: 2212
call: 2212
call: 72
call: 2231
call: 2231
call: 2205
call: 2182
call: 2221
call: 2231
call: 2165
call: 2231
call: 2231
call: 2231
call: 2231
call: 2228
call: 2231
call: 2231
call: 2242
call: 2038
call: 2242
call: 2242
call: 2085
call: 2065
call: 2242
call: 2047
call: 2242
call: 2030
call: 2242
call: 2242
call: 2135
call: 2243
call: 2243
call: 2243
call: 2241
call: 2243
call: 1893
call: 2243
call: 2243
NEW 2243
call: 1229
call: 1469
call: 1469
call: 792
call: 1340
call: 1413
call: 1469
call: 1501
call: 1501
call: 1501
call: 74
call: 1501
call: 1501
call: 1501
call: 1501
call: 1501
call: 1501
call: 1005
call: 1501
call: 1014
call: 1501
call: 2085
call: 2085
call: 2085
call: 2085
call: 2085
call: 2112
call: 2112
call: 2157
call: 2157
call: 2157
call: 0
call: 2157
call: 2172
call: 2172
call: 2172
call: 2172
call: 2188
call: 2131
call: 2126
call: 2188
call: 2129
call: 2188
call: 2188
call: 24
call: 2176
call: 2161
call: 2188
call: 2198
call: 2149
call: 2041
call: 2073
call: 2198
call: 2086
call: 2198
call: 2133
call: 2156
call: 2198
call: 2097
call: 2198
call: 2146
call: 2067
call: 2198
call: 2239
call: 2216
call: 2246
call: 2246
call: 2237
call: 144
call:


 SCORE: 2034
64
RRF CWF FFF WCR FCC CCR CWR WCC RFW FWW RWW RFR RRC FFR FFW CFC FRR RCC CCW CRF WWC FWC FRW RWF FFC WWW FWR WRR WWR WCF WRW WRC FRC CWW FCR FRF RRW RRR WCW FCW RWR WFW RWC FWF CFR WWF RFF RCF CRR CRC CWC RFC WFC RCW CFF CCF FCF RCR WFF WFR CCC WRF CRW CFW

SCORE: 2118
64
RRF CWF FFF WCR FCC CCR CWR WCC RFW FWW RWW RFR RRC FFR FFW CFC FRR RCC CCW CRF WWC FWC FRW RWF FFC WWW FWR WRR WRW WCF CCF WFC FRC CWW FCR FRF FCW RRR CFW RRW RWR WFW RWC FWF CFR WWF RFF RCF CRR CRC CWC RFC WRC RCW CFF WWR FCF RCR WFF WFR CCC WRF CRW WCW

SCORE: 2191
64
RRF CWF FFF WCR FCC CCR CWR WCC RFW FWW RWW RFR RRC FFR FFW CFC FRR RCC CCW CWW WWC FWC FRW RWF FFC WWW FWR WRR WRW WCF CCF WFC FRC CRF FCR FRF FCW RRR CFW RRW RWR WFW RWC FWF CFR WWF RFF RCF CRR CRC CWC RFC WRC RCW CFF WWR FCF RCR WFF WFR CCC WRF CRW WCW

SCORE: 2212
64
WCW WCC CWR WWR RWW WRC FCC CRR RRC CCR FFF CWC RWF RWR CFF CRC FFR RFW RCC CRW RFC FCF FRW FWF RRW FWW CFW WFF WCR WWC FFC FRR CFR RWC CCC FWR WCF RFF RCW WFC WWF RCF WFR FCR FRC CCW WFW CWW RRR CFC RRF FRF WWW CCF CWF CRF WRF WRR FCW FWC RCR RFR FFW WRW

SCORE: 2231
64
WCW WCC CWR CRC RWW CWF FCC CRR RRC CCR FFF CWC RWF RWR CFF WWR FFR RFW RCC CRW RFC FCF FRW FWF RRW FWW CFW WFF WCR WWC FFC FRR CFR RWC CCC FWR WCF RFF RCW WFC WWF RCF WFR FCR FRC CCW WFW CWW RRR CFC RRF FRF WWW CCF WRC CRF WRF WRR FCW FWC RCR RFR FFW WRW

SCORE: 2242
64
WCW WCC CWR CRC FCF CWF FCC CRR RRC WCR FFF CWC RWF WRC CFF WWR FFR RFW RCC CRW CCW RWW FRW FWF RRW FWW RCW WFF CCR WWC CFW FRR CFR RWC RWR FWR WCF RFF FFC WFC WWF FCW WFR RRF FRC RFC WFW CWW RRR CFC FCR FRF WWW CCF CCC CRF WRF WRR RCF FWC RCR RFR FFW WRW

SCORE: 2243
64
WCW WCC RRW FRC FCF CWF FCC CRR RRC CCW FFF CWC RWF WRC CFF WWC FFR RFW RCC FCR FFW CRW FRW FWF CWR FWW RCW WFF CCR WWR CFW FRR CFR RWC RWR FWR WCF RFF FFC WFC WWF FCW WFR RRF CRC RFC WFW CWW RRR CFC RWW FRF WWW CCF CCC CRF WRF WRR RCF FWC RCR RFR WCR WRW

SCORE: 2246
64
FCR RCC FRR WRR WWC CRF RRW RFF FWW RWW FRW CRW RCR WCC CWC CCR FRC FFW WWR WCW WFW FFC CRC WWW RWC WRC RWR RFW WFC CFC FRF FCF RFC RRF RFR CWR WWF FWC FFF FWR CCF WCR CFW RCF WFF RRC CCC CRR CWF RRR CFR WRW CWW CCW WFR CFF FCC RWF WCF FCW RCW WRF FFR FWF


 */