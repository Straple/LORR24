#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>

// ssh -i ../abc egor@51.250.101.48

// scp -i ../abc -r * egor@51.250.101.48:/home/egor

// ./compile.sh

// ./build/lifelong -i ./example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 300 -p 1800000

// ./build/lifelong -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

// ./build/lifelong -i ./example_problems/my.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

// python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

// -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 500 -p 1800000

//#define ENABLE_DEFAULT_PLANNER

//#define ENABLE_DEFAULT_SCHEDULER

#define ENABLE_ASSERT

#define ENABLE_HEURISTIC_MATRIX

#define ENABLE_PIBT

#define ENABLE_DHM

// при завершении программы вызывает tools.cpp::build_meta_info в driver.cpp
//#define BUILD_META_INFO

#define ENABLE_PRINT_LOG

#define ENABLE_PIBTS_ANNEALING

#define ENABLE_PIBTS_TRICK

//#define ENABLE_GG_SOLVER

static constexpr uint32_t MAX_CONST = -1;

static constexpr uint32_t THREADS = 6;

static constexpr uint32_t PLANNER_DEPTH = 3;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = 1000;

static constexpr uint32_t DHM_REBUILD_TIMELIMIT = 500;

static constexpr uint32_t DHM_REBUILD_COUNT = MAX_CONST;

static constexpr uint32_t SCHEDULER_REBUILD_DP_TIME = 100;

static constexpr uint32_t SCHEDULER_TRIV_SOLVE_TIME = 200;

//-i ./example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 300 -p 100000000

static constexpr uint32_t INVALID_DIST = 0;

uint32_t &get_unique_id();

#define ENABLE_FILEPRINT

struct Printer {
    [[nodiscard]] std::ofstream &get() const;
};

template<typename T>
Printer operator<<(Printer printer, const T &value) {
#ifdef ENABLE_FILEPRINT
    printer.get() << value;
    std::cout << value;
#else
    std::cout << value;
#endif
    return printer;
}

/*
6 cores
PIBTS_STEPS = 1000
ENABLE_ALL
call(0): 2294, 19.5517s
call(1): 4079, 56.2201s
call(2): 5217, 66.1385s
call(3): 5530, 65.8229s
call(4): 5072, 112.691s
call(5): 4322, 189.266s
total: 26514
*/

/*
call(0): 2405, 12.1126s
call(1): 4211, 17.6213s
call(2): 5260, 24.673s
call(3): 5762, 34.4333s
call(4): 5285, 98.5209s
call(5): 4302, 174.456s
total: 27225
*/

/*
32 cores
use PIBTS
if PIBTS_STEPS enable, then PIBTS_STEPS=1000
if disable ENABLE_PIBTS_TRICK and PIBTS_STEPS=0, then PIBTS = PIBT2

1) DHM + PIBTS_STEPS + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2403, 11.3534s
call(1): 4184, 16.4483s
call(2): 5236, 23.3809s
call(3): 5797, 32.4601s
call(4): 5342, 87.6157s
call(5): 4193, 405.141s
total: 27155

=======================

2) DHM + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2350, 7.18267s
call(1): 3975, 11.0802s
call(2): 4827, 15.4018s
call(3): 5028, 21.9001s
call(4): 4634, 55.2668s
call(5): 3846, 150.983s
total: 24660

3) DHM + PIBTS_STEPS + ENABLE_SCHEDULER_TRICK
call(0): 2404, 12.0467s
call(1): 4192, 17.2747s
call(2): 5221, 23.7821s
call(3): 5624, 32.6402s
call(4): 4960, 86.9758s
call(5): 3764, 247.762s // TODO: попробовать увеличить PIBTS_STEPS так, чтобы сравнился с (1)
total: 26165

4) DHM + PIBTS_STEPS + ENABLE_PIBTS_TRICK
call(0): 2127, 12.2567s
call(1): 3704, 17.7565s
call(2): 4575, 24.9318s
call(3): 4951, 37.8259s
call(4): 4198, 169.524s
call(5): 3350, 495.446s
total: 22905

5) PIBTS_STEPS + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2372, 21.2071s
call(1): 3987, 37.6914s
call(2): 4190, 121.831s
call(3): 3471, 409.616s
call(4): 3092, 957.415s
call(5): 2604, 1967.65s
total: 19716

=======================

# тут все потоки выдают одинаковое решение, так как не используется рандом: disable ENABLE_PIBTS_TRICK + disable PIBTS_STEPS
6) DHM + ENABLE_SCHEDULER_TRICK
call(0): 2261, 7.07915s
call(1): 3701, 10.8508s
call(2): 4275, 14.6964s
call(3): 4176, 19.5032s
call(4): 3630, 34.707s
call(5): 3127, 52.4499s
total: 21170

7) DHM + PIBTS_STEPS
call(0): 2122, 12.1423s
call(1): 3699, 17.3189s
call(2): 4554, 23.6698s
call(3): 4959, 33.3806s
call(4): 4078, 111.868s
call(5): 3240, 234.339s
total: 22652

8) DHM + ENABLE_PIBTS_TRICK
call(0): 2052, 7.17383s
call(1): 3520, 11.0261s
call(2): 4192, 15.2919s
call(3): 4362, 21.9761s
call(4): 3773, 78.2991s
call(5): 3100, 233.486s
total: 20999

9) PIBTS_STEPS + ENABLE_PIBTS_TRICK
call(0): 2104, 21.1304s
call(1): 3387, 42.1391s
call(2): 3072, 208.643s
call(3): 2819, 510.713s
call(4): 2413, 1190.89s
call(5): 2206, 1807.67s
total: 16001

10) PIBTS_STEPS + ENABLE_SCHEDULER_TRICK

11) ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK

=======================

12) DHM
call(0): 1762, 6.95711s
call(1): 3227, 10.6797s
call(2): 3531, 14.3174s
call(3): 3355, 19.7687s
call(4): 2999, 33.3046s
call(5): 2511, 66.3698s
total: 17385

13) ENABLE_PIBTS_TRICK

14) ENABLE_SCHEDULER_TRICK
call(0): 2065, 4.87735s
call(1): 3009, 12.0905s
call(2): 2628, 66.2085s
call(3): 2585, 165.536s
call(4): 2352, 258.666s
call(5): 2161, 493.386s
total: 14800

15) PIBTS_STEPS

=======================

16)
call(0): 1355, 16.1625s
call(1): 2503, 14.6241s
call(2): 2254, 66.3312s
call(3): 2065, 172.373s
call(4): 1886, 406.353s
call(5): 1792, 586.784s
total: 11855


17) MAPFPLANNER + мой шедулер
call(0): 1840, 181.398s
call(1): 2877, 181.792s
call(2): 2743, 182.008s
call(3): 2650, 182.18s
call(4): 2395, 182.558s
call(5): 2019, 182.778s
total: 14524

18) MAPFPLANNER + их шедулер
call(0): 1756, 181.116s
call(1): 2773, 181.62s
call(2): 2636, 181.991s
call(3): 2557, 182.399s
call(4): 2229, 182.812s
call(5): 1838, 183.061s
total: 13789

19) MAPFPLANNER + мой шедулер + ENABLE_SCHEDULER_TRICK
call(0): 2107, 181.495s
call(1): 3392, 181.779s
call(2): 3737, 181.739s
call(3): 3239, 182.445s
call(4): 2868, 182.69s
call(5): 2330, 182.785s
total: 17673

20) MAPFPLANNER + мой шедулер + ENABLE_SCHEDULER_TRICK + DHM
call(0): 2096, 181.353s
call(1): 3361, 181.56s
call(2): 2944, 181.956s
call(3): 3117, 182.294s
call(4): 2935, 182.608s
call(5): 2440, 182.862s
total: 16893

*/

/*
call(0): 2352, 19.4394s
call(1): 2325, 20.138s
call(2): 4137, 50.8091s
call(3): 4081, 57.113s
call(4): 5238, 51.5664s
call(5): 5177, 57.8458s
call(6): 5761, 67.9453s
call(7): 5543, 67.6235s
call(8): 4973, 140.395s
call(9): 5207, 111.707s
call(10): 4078, 304.588s // PIBTS_STEPS = 0: 165.8s 3802, если без rnd.get_d() < 0.8, то 91.5006s 3075
call(11): 4004, 306.231s // TODO: тут оочень медленно с PIBTS_STEPS=1000, попробовать поменять
total: 52876
TODO: попробовать поменять try_build так, чтобы он стремился на начальных агентах улучшать скор,\
 а если это не так, то с некоторой вероятностью он выйдет

TODO:
-7) Взяли клетку. Запустили Дейкстру. Взяли среднее значение стоимости пути до всех клеток.
Сделали так для всех проходимых клеток на карте.
Нашли среди всех клеток максимальное среднее значение.
Каждой клетке вместо 1 присвоили значение равное максимуму деленному на среднее значение стоимости пути для текущей клетки.
-6) // TODO: у to_r может быть приоритет ниже чем у r
// но to_r уже построен, потому что был какой-то x, который имел приоритет больше чем r
// и этот x построил to_r
-5) запустить решение на прошлогодних данных, сравнить с топами
-4) отправить посылку с parallel_build, попробовать включить annealing
-3) отправить посылку где шедулер использует только HM
-2) я сейчас запускаю GG с весами 500 изначально. Может быть сделать их маленькими: <10
-1) в PIBTS попробовать заложить в стоимость операции потенциальный конфликт при перестроении
0) попробовать поддерживать N разных PIBTS состояний. Потоки будут пытаться равномерно их считать
1) Разобраться, почему в какой-то момент PIBTS очень долго работает. Выписать лог рекурсии, понять что он делает.
   Возможно к этому причастно какое-то удивительное состояние. Может быть есть роботы, которые близки к своему target.
   И они как-то плохо влияют на рекурсию. Пока я сам разбирался, то понял, что если ограничивать глубину рекурсии,
   то получается очень плохо и медленно. Так как он не находит результат и расширяется в дереве рекурсии,
   что ведет к росту сложности
2) Улучшить рекурсию в PIBTS. Как время так и качество результатов
3) Возможно, включить parallel_build. Когда сразу много потоков пытаются улучшить одно состояние (так делали победители).
4) Про GG: для каждой позиции на карте можно сделать число = -1, если не определено, 0-3 -- рекомендуемые направления движения.
   Если робот делает действие, которое не сходится с желаемым направлением (поворачивает в другую сторону, идет в другом направлении).
   То за это штраф некоторым числом, которое также подбирается в GraphGuidanceSolver. Если -1, то просто обычный вес.
   Если он идет в правильном направлении, то тоже обычный вес, который тоже подбирается в GGS.
   Далее эта штука когда нужно дать в lifelong, то GGS преобразует его в стандартную схему: weight[pos][dir][action].
   И подает на вход.

*)  UPD: что-то не очень получилось
    (static_cast<int32_t>(robots.size()) - weight[r])
    может быть слишком большая разница между первым и последним
    может быть взять sqrt()
    может быть попробовать поделить на dist
    попробовать всякие разные веса
*)  UPD: что-то не очень получилось
    "16 FFF FFW FWF FWW WFF WFW WWF FCF FRF RFF CFF RFW CFW RRF RWF CWF"
    попробовать добавить количество операций разной длины: F, FF, CF и прочее.
*/
