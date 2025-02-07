
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
//без DHM
//time:  466.095s
//tasks: 35056
//enable try_rebuild_neigbors
//tasks: 35025

//tasks: 34067, SCHEDULER_LNS_TIME = 100

//tasks: 34422, SCHEDULER_LNS_TIME = 0
//tasks: 22155 без GG

// 6256 -> 6365

// cutoffTime: 0.95, score: 6101