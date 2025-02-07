/*
Порядок выполнения тестов:
WAREHOUSE
SORTATION
RANDOM-05
RANDOM-04
RANDOM-03
RANDOM-02
RANDOM-01
GAME
CITY-02
CITY-01

он идет в обратном порядке от списка на сайте
*/


/*
ssh -i ../abc egor@51.250.101.48

scp -i ../abc -r * egor@51.250.101.48:/home/egor

./compile.sh

./build/lifelong -i ./example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 300 -p 1800000

./build/lifelong -i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

./build/lifelong -i ./example_problems/my.domain/random_32_32_20_100.json -o test.json -s 1000 -t 300 -p 1800000

python3 PlanViz/script/run2.py --map example_problems/random.domain/maps/random-32-32-20.map --plan test.json --end 1000

-i ./example_problems/random.domain/random_32_32_20_100.json -o test.json -s 1000 -t 500 -p 1800000
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
32 cores
ENABLE_ALL

call(0): 2386, 80.9281s
call(1): 4277, 81.1942s
call(2): 5486, 81.2495s
call(3): 6313, 81.4385s
call(4): 6178, 81.8762s
call(5): 4388, 81.8032s
total: 29028
*/

/*
веса операций + без DHM
call(0): 1401, 48.7164s
call(1): 2536, 48.8561s
call(2): 3645, 65.4042s
call(3): 3885, 82.4048s
call(4): 5512, 165.276s
total: 16979

веса операций + DHM
power * power
call(0): 1394, 48.6156s
call(1): 2488, 48.7558s
call(2): 4742, 65.2078s
call(3): 4128, 81.9905s
call(4): 5235, 164.159s
total: 17987
power
call(0): 1416, 48.6326s
call(1): 2524, 48.7505s
call(2): 4985, 65.221s
call(3): 4086, 81.985s
call(4): 5010, 164.162s
total: 18021

call(0): 1421, 48.6268s
call(1): 2527, 48.7458s
call(2): 4901, 65.2095s
call(3): 4076, 81.9673s
call(4): 5178, 164.172s
total: 18103

DHM
call(0): 1435, 48.6835s
call(1): 2539, 48.7773s
call(2): 5086, 65.3003s
call(3): 3629, 82.0164s
call(4): 4688, 164.299s
total: 17377
*/

/*
call(0): 10173, 1525.63s
call(1): 46233, 3524.64s
call(2): 2621, 477.479s
call(3): 1611, 477.273s
call(4): 7159, 956.207s
call(5): 155049, 5209.88s ENTRY TIMEOUT
call(6): 5084, 956.684s
call(7): 6047, 1916.77s
call(8): 10174, 5087.78s
call(9): 102886, 5250.45s ENTRY TIMEOUT
total: 347037
*/

// RESULTS(32): (17994, 76, 13080) (17628, 77, 7410) (17537, 77, 8672) (17391, 77, 8853) (17357, 77, 8092) (17310, 77, 8057) (17282, 77, 6763) (17243, 77, 8825) (17235, 77, 10967) (17221, 76, 5428) (17203, 77, 5671) (17201, 77, 7013) (17191, 77, 6471) (17154, 76, 8447) (17151, 77, 6658) (17091, 77, 8786) (17088, 77, 6610) (17085, 76, 5958) (17056, 77, 4592) (17050, 77, 4277) (17037, 76, 8548) (16986, 76, 6697) (16980, 77, 5532) (16974, 77, 7466) (16935, 76, 8563) (16865, 76, 9151) (16836, 77, 11389) (16795, 76, 6616) (16690, 76, 3613) (16588, 76, 1543) (16503, 77, 6500) (15931, 76, 4561)
// without DHM:
// 4585(5184)

//RESULTS(32): (80455, 68, 32225) (80391, 68, 32175) (80320, 68, 31878) (80220, 67, 32595) (80220, 67, 34368) (80203, 67, 32043) (80105, 68, 34269) (80093, 67, 32866) (80062, 67, 31846) (79994, 68, 34212) (79978, 67, 30377) (79956, 67, 34806) (79917, 67, 31350) (79872, 68, 35397) (79827, 68, 35820) (79774, 68, 33441) (79626, 67, 34348) (79615, 67, 33771) (79607, 67, 32130) (79594, 67, 33524) (79570, 68, 30562) (79534, 67, 28237) (79519, 68, 36541) (79493, 68, 32319) (79473, 67, 35030) (79460, 68, 32490) (79425, 68, 29493) (79292, 68, 35179) (79277, 68, 32649) (79160, 68, 30488) (79124, 67, 36114) (78917, 68, 27977)
// DHM:
// 6322(6921)

//-i example_problems/warehouse.domain/warehouse_large_5000.json -o test.json -s 1000 -t 500 -p 1000000000
//459.301s
//без DHM
//tasks: 19359
//score: 96726.9
//
//DHM:
//tasks: 19447
//score: 97174.2

//-i example_problems/warehouse.domain/warehouse_large_10000.json -o test.json -s 1000 -t 500 -p 1000000000
//time:  466.095s
//tasks: 35056 -> 35775
//
//tasks: 34067, SCHEDULER_LNS_TIME = 100
//tasks: 34422, SCHEDULER_LNS_TIME = 0
//tasks: 22155 без GG

//-i example_problems/game.domain/brc202d_6500.json -o test.json -s 1000 -t 500 -p 1000000000
//ENABLE_SMART_PLANNER: 6758
//MY: 6524


// (revealed tasks)
//6642 -> 7045 -> 7097
//6979
/*
call(0): 2513, 11.6532s
call(1): 4728, 16.9672s
call(2): 5987, 29.2818s
call(3): 6291, 53.7082s
call(4): 5528, 80.692s
call(5): 4276, 81.9039s
call(6): 3441, 82.2837s
call(7): 2374, 82.4319s
total: 35138

call(0): 2553, 11.2305s
call(1): 4702, 17.3299s
call(2): 6045, 28.9299s
call(3): 6221, 57.6373s
call(4): 4919, 81.3932s
call(5): 4157, 81.9923s
call(6): 3405, 82.3248s
call(7): 2300, 82.537s
total: 34302

call(0): 2517, 80.9272s
call(1): 4566, 81.146s
call(2): 5629, 81.4092s
call(3): 6011, 81.6299s
call(4): 5636, 81.8592s
call(5): 5288, 82.0845s
call(6): 4873, 82.065s
call(7): 3083, 82.2727s
total: 37603
*/

//6882

//4334 ->

//-i example_problems/game.domain/brc202d_6500.json -o test.json -s 1000 -t 500 -p 1000000000
// 16362 -> 17191 (max assigned 5000) -> 17762 (4500) -> 19036 (3000) -> 19451(3500)

//Timestep: 603
//free robots: 1563
//free tasks: 4813
//SchedulerSolver::rebuild_dp: 1563/1563 (100%), 138.208ms
//SchedulerSolver::triv_solve: 313.278us
//SchedulerSolver::solve: 1.49905e+09->1.49905e+09, 0, 4.183us
//Scheduler: 139.101ms
//Scheduler robots init: 5001/6500 (76.9385%)
//build used: 17.5955ms
//build edges: 5.77299ms
//build poses: 10.8194ms
//init neighbors: 39.6544ms
//RESULTS(32): (1.70737e+06, 223, 9152) (1.70411e+06, 223, 8077) (1.70388e+06, 223, 8471) (1.70318e+06, 224, 8508) (1.70125e+06, 224, 9290) (1.70079e+06, 223, 7589) (1.69971e+06, 224, 7749) (1.69853e+06, 224, 8827) (1.69813e+06, 223, 7954) (1.69661e+06, 223, 7868) (1.69641e+06, 223, 8005) (1.69344e+06, 223, 6825) (1.69336e+06, 223, 8294) (1.69326e+06, 223, 7596) (1.69083e+06, 224, 8242) (1.68964e+06, 223, 6753) (1.68917e+06, 223, 7865) (1.68699e+06, 224, 7739) (1.68643e+06, 223, 7130) (1.68549e+06, 224, 6895) (1.68308e+06, 224, 7676) (1.68084e+06, 223, 7896) (1.68071e+06, 223, 6214) (1.67968e+06, 224, 6523) (1.67805e+06, 223, 6715) (1.67764e+06, 224, 6342) (1.67617e+06, 223, 7341) (1.67516e+06, 223, 6328) (1.67496e+06, 224, 6577) (1.67059e+06, 224, 6967) (1.66697e+06, 224, 5343) (1.66218e+06, 223, 5887)
//best: 1.70737e+06
//Desires:
//0 WWW 11140 0
//1 FFF 2183304 1687282200
//2 FFW 337284 193260920
//3 FWW 234007 86138770
//4 FRF 99703 51127350
//5 FCF 86237 44323250
//6 RFW 116028 32912160
//7 CFW 101792 26323040
//8 RFF 129238 52080320
//9 CFF 88921 40667040
//10 RRF 51136 14844760
//11 WFW 126098 41426540
//12 FWF 111877 64489190
//13 WFF 121019 65099740
//14 WWF 51307 18981870
//15 RWF 40247 13153740
//16 CWF 36662 11520840
//Planner: 302.315ms
//F: 4193
//R: 890
//C: 617
//W: 800
//N: 0
//Entry time: 453.386ms
//Total time: 278.647s
//[2025-02-07 21:03:48.646867] [0x000075c1754965c0] [info]    [timestep=603] planner returns
//[2025-02-07 21:03:48.652886] [0x000075c1754965c0] [info]    [timestep=604] Agent 121 finishes task 5408
//[2025-02-07 21:03:48.652931] [0x000075c1754965c0] [info]    [timestep=604] Agent 486 finishes task 14948
//[2025-02-07 21:03:48.652971] [0x000075c1754965c0] [info]    [timestep=604] Agent 996 finishes task 311
//[2025-02-07 21:03:48.653038] [0x000075c1754965c0] [info]    [timestep=604] Agent 2086 finishes task 7069
//[2025-02-07 21:03:48.653057] [0x000075c1754965c0] [info]    [timestep=604] Agent 2261 finishes task 12771
//[2025-02-07 21:03:48.653166] [0x000075c1754965c0] [info]    [timestep=604] Agent 4133 finishes task 9389
//[2025-02-07 21:03:48.653184] [0x000075c1754965c0] [info]    [timestep=604] Agent 4314 finishes task 8252
//[2025-02-07 21:03:48.653202] [0x000075c1754965c0] [info]    [timestep=604] Agent 4506 finishes task 5348
//[2025-02-07 21:03:48.653252] [0x000075c1754965c0] [info]    [timestep=604] Agent 5290 finishes task 10272
//[2025-02-07 21:03:48.653266] [0x000075c1754965c0] [info]    [timestep=604] Agent 5423 finishes task 801
//[2025-02-07 21:03:48.653337] [0x000075c1754965c0] [info]    Task 15474 is revealed
//[2025-02-07 21:03:48.653344] [0x000075c1754965c0] [info]    Task 15475 is revealed
//[2025-02-07 21:03:48.653354] [0x000075c1754965c0] [info]    Task 15476 is revealed
//[2025-02-07 21:03:48.653358] [0x000075c1754965c0] [info]    Task 15477 is revealed
//[2025-02-07 21:03:48.653366] [0x000075c1754965c0] [info]    Task 15478 is revealed
//[2025-02-07 21:03:48.653372] [0x000075c1754965c0] [info]    Task 15479 is revealed
//[2025-02-07 21:03:48.653380] [0x000075c1754965c0] [info]    Task 15480 is revealed
//[2025-02-07 21:03:48.653386] [0x000075c1754965c0] [info]    Task 15481 is revealed
//[2025-02-07 21:03:48.653393] [0x000075c1754965c0] [info]    Task 15482 is revealed
//[2025-02-07 21:03:48.653399] [0x000075c1754965c0] [info]    Task 15483 is revealed