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

#define ENABLE_ASSERT

#define ENABLE_HEURISTIC_MATRIX

#define ENABLE_PIBT

#define ENABLE_DHM

// при завершении программы вызывает tools.cpp::build_meta_info в driver.cpp
#define BUILD_META_INFO

#define ENABLE_PRINT_LOG

#define ENABLE_PIBTS_ANNEALING

#define ENABLE_PIBTS_TRICK

#define ENABLE_SCHEDULER_TRICK

//#define ENABLE_GG_SOLVER

static constexpr uint32_t MAX_CONST = -1;

static constexpr uint32_t THREADS = 32;

static constexpr uint32_t PLANNER_DEPTH = 3;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = 1000;

static constexpr uint32_t DHM_REBUILD_TIMELIMIT = MAX_CONST;

static constexpr uint32_t DHM_REBUILD_COUNT = MAX_CONST;

static constexpr uint32_t SCHEDULER_REBUILD_DP_TIME = MAX_CONST;

static constexpr uint32_t SCHEDULER_TRIV_SOLVE_TIME = MAX_CONST;

struct EPlanner;   // мой алгоритм
struct MAPFPlanner;// их алгоритм
using PLANNER = EPlanner;

struct MyScheduler;  // мой алгоритм
struct TaskScheduler;// их алгоритм
using TASKSHEDULLER = MyScheduler;

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
PIBTS_STEPS=1000
use PIBTS
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
call(5): 3764, 247.762s
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


*/


/*
OLD

1) DHM + GG + PIBTS + PIBTS_STEPS=1000 + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2292, 12.0526s
call(1): 4100, 17.7083s
call(2): 5189, 24.8347s
call(3): 5659, 35.6665s
call(4): 5100, 109.694s
call(5): 4233, 292.406s
total: 26573

2) DHM + GG + PIBTS + PIBTS_STEPS=1000 + ENABLE_SCHEDULER_TRICK
call(0): 2292, 11.3482s
call(1): 4071, 16.4324s
call(2): 5150, 22.311s
call(3): 5586, 31.2824s
call(4): 4720, 86.8913s
call(5): 3705, 208.457s
total: 25524

3) DHM + GG + PIBTS + PIBTS_STEPS=0 + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2260, 6.86378s
call(1): 3904, 10.6975s
call(2): 4787, 14.5094s
call(3): 4964, 20.2558s
call(4): 4346, 73.3642s
call(5): 3683, 191.437s
total: 23944

4) DHM + GG + PIBTS + PIBTS_STEPS=1000 + ENABLE_PIBTS_TRICK
call(0): 2042, 11.3738s
call(1): 3662, 16.7337s
call(2): 4585, 23.0989s
call(3): 5049, 32.9808s
call(4): 4212, 120.688s
call(5): 3313, 358.855s
total: 22863

5) DHM + PIBTS + PIBTS_STEPS=1000 + ENABLE_PIBTS_TRICK + ENABLE_SCHEDULER_TRICK
call(0): 2403, 11.3534s
call(1): 4184, 16.4483s
call(2): 5236, 23.3809s
call(3): 5797, 32.4601s
call(4): 5342, 87.6157s
call(5): 4193, 405.141s
total: 27155

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

PIBTS_STEPS = 1000
258.399s 3667
175.168s 3834

TODO:
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
5) Собрать результаты по всем подмножествам технологий, которые сейчас есть: PIBT, PIBT2, PIBTS, DHM, GG
   Это нужно, чтобы четко знать: какие технологии и сколько дают и в какой связке. Может какую-то технологию можно улучшить
*/
