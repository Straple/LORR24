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

#define ENABLE_TRIVIAL_SCHEDULER

//#define ENABLE_ASSERT

#define ENABLE_HEURISTIC_MATRIX

#define ENABLE_PIBT

#define ENABLE_DHM

// при завершении программы вызывает tools.cpp::build_meta_info в driver.cpp
#define BUILD_META_INFO

#define ENABLE_PRINT_LOG

#define ENABLE_PIBTS_ANNEALING

#define ENABLE_PIBTS_TRICK

//#define ENABLE_GG_SOLVER

static constexpr uint32_t MAX_CONST = 10'000'000;

static constexpr uint32_t THREADS = 32;

static constexpr uint32_t PLANNER_DEPTH = 3;

// if -1, then use timer
// else use steps, without timer
static constexpr uint32_t PIBTS_STEPS = 1000;

static constexpr uint32_t DHM_REBUILD_TIMELIMIT = 400;

static constexpr uint32_t DHM_REBUILD_COUNT = MAX_CONST;

static constexpr uint32_t SCHEDULER_REBUILD_DP_TIME = MAX_CONST;

static constexpr uint32_t SCHEDULER_TRIV_SOLVE_TIME = MAX_CONST;

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
-i Data2023/Main/random.domain/MR23-I-03.json -o test.json -s 500 -t 200 -p 100000000
Total time: 90.6059s
score: 2573 -> 2631
*/

/*
call(0): 10170 / 10385, 1549.94s
call(1): 47649 / 49186, 3533.54s
call(2): 2547 / 3042, 490.689s
call(3): 1530 / 1741, 490.636s
call(4): 7026 / 7432, 981.584s
call(5): 180743 / 197275, 5103.04s
call(6): 5235 / 5914, 982.136s
call(7): 6580 / 6059, 1964.96s
call(8): 12316 / 28954, 5052.89s
call(9): 164905 / 194677, 5069.29s
total: 438701 / 504665
*/

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
32 cores
ENABLE_ALL

call(0): 2421, 81.0339s
call(1): 4244, 81.1961s
call(2): 5339, 81.3922s
call(3): 6179, 81.5677s
call(4): 5924, 81.7985s
call(5): 4400, 82.1342s
total: 28507

call(0): 2410, 81.0495s
call(1): 4248, 81.2473s
call(2): 5372, 81.4135s
call(3): 6145, 81.5723s
call(4): 5718, 81.9133s
call(5): 4339, 82.1599s
total: 28232
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
TODO:
!!!) мы должны были выяснить что сделать за эти WW. Мы должны перебрать CWW, RWW, RRW и взять лучший и сделать его
-8) сделать более умный вес задачи у шедулера. Например: для каждой задачи найти самую ближайшую на ее конце и дать этому некоторый вес
    таким образом он будет не только брать самую близкую задачу, но еще и думать чуток наперед.
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
5) про PIBT:
вот мы хотим построить робота r. выбрали действие и видим там робота to_r, который нам мешает. Я сношу его, добавляю путь r и рекурсивно строю to_r. А почему бы не сделать так: рассмотреть пары (операция робота r, операция to_r). И конечно, чтобы они не коллизили друг с другом, но могут коллизить с другими роботами. И дать им приоритет = некоторой метрике об этих двух операциях. Отсортировать это и пройтись. Тут мы уже берем и ставим два пути, и возможно коллизим с третьим роботом, для него рекурсивно построить

ну или вообще строим для робота r. Переберем все его действия. если действие коллизит с другим роботом r2. То переберем еще и его действия. Но этот набор может коллизить с другим роботом r3. Которого мы уже рекурсивно построим. В итоге все это сортируем по весу, обходим в этом порядке и выполняем. Так мы даже сможем построить, если действие робота r коллизит с двумя роботами

Есть уже такое. Правда делалось это для оптимального алгоритма CBS, который не скейлится нифига)
Называется meta-agents.
Когда мы объединяем нескольких агентов в одну сущность и работаем с ним как с одним.

Таким образом мы можем решить вопрос с запуском рекурсии при конфликте с двумя агентами

Сейчас мы не понимаем на кого из них запустить рекурсию, а так мы запускаем её на одного мета-агента.

У такого подхода есть очевидное ограничение. Количество комбинаций действий растёт экспоненциально. Учитывая, что мы рассматриваем 16 действий, мета агент из 3х агентов - это уже 2048 возможных комбинаций.







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
