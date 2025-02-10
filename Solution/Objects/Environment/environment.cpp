#include <Objects/Environment/environment.hpp>

#include <Scheduler/journey_graph.hpp>
#include <Scheduler/scheduler.hpp>
#include <Tools/tools.hpp>
#include <settings.hpp>

#include "heuristics.h"
#include "planner.h"

void init_default_planner(SharedEnvironment &env) {
#if defined(ENABLE_DEFAULT_PLANNER) || defined(ENABLE_DEFAULT_SCHEDULER)
    ETimer timer;
    DefaultPlanner::initialize(100000, &env);
    for (uint32_t pos = 1; pos < get_map().get_size(); pos++) {
        if (!get_map().is_free(pos)) {
            continue;
        }
        for (uint32_t to = 1; to < get_map().get_size(); to++) {
            if (!get_map().is_free(to)) {
                continue;
            }
            DefaultPlanner::get_h(&env, pos - 1, to - 1);
        }
    }
#ifdef ENABLE_PRINT_LOG
    Printer() << "init_default_planner: " << timer << '\n';
#endif

#endif
}

void init_environment(SharedEnvironment &env) {
    static bool already_init = false;
    if (already_init) {
        return;
    }
    already_init = true;

    // update map type
    {
        if (env.map_name == "warehouse_large.map") {
            get_map_type() = MapType::WAREHOUSE;
        } else if (env.map_name == "sortation_large.map") {
            get_map_type() = MapType::SORTATION;
        } else if (env.map_name == "brc202d.map") {
            get_map_type() = MapType::GAME;
        } else if (env.map_name == "Paris_1_256.map") {
            get_map_type() = MapType::CITY;
        } else if (env.map_name == "random-32-32-20.map") {
            get_map_type() = MapType::RANDOM;
        } else {
            get_map_type() = MapType::NONE;
            FAILED_ASSERT("undefined map");
        }
    }

    get_map() = Map(env);
#ifdef ENABLE_GG_SOLVER
    // read GraphGuidance
    {
        std::ifstream input("Tmp/gg" + std::to_string(get_unique_id()));
        input >> get_gg();
    }
    // read operations weights
    {
        std::ifstream input("Tmp/opw" + std::to_string(get_unique_id()));
        uint32_t k = 0;
        input >> k;
        get_operations_weights().resize(k);
        for (int &x: get_operations_weights()) {
            input >> x;
        }
    }
#else
    Printer().get() = std::ofstream("printer.txt");
    get_guidance_map() = GuidanceMap(get_map_type(), get_map());
    // warehouse bad guidance map
    if (get_map_type() == MapType::RANDOM
        // TODO: GuidanceMap для warehouse довольно плох
        //|| get_map_type() == MapType::WAREHOUSE
            ) {
        //Printer() << "GuidanceMap\n";
        // 52128
        get_gg() = GraphGuidance(get_guidance_map());
    } else {
        //Printer() << "without GuidanceMap\n";
        // 68650 or 73979
        get_gg() = GraphGuidance(env);
    }

    // init operations weights
    /*{
        std::stringstream input("45 -7 38 -18 11 72 -34 72 -4 52 31 38 17 -32 7 45 24 48 -35 49 19 -75 0 25 -54 -26 2 -9 -71 -22 -2 38 29 18 -72 16 46 0 9 1 21 -7 15 33 19 -1");
                //("17 -200 20 20 10 60 70 30 10 40 40 20 20 50 50 110 130 150");
        uint32_t k = 0;
        input >> k;
        get_operations_weights().resize(k);
        for (int &x: get_operations_weights()) {
            input >> x;
        }
    }*/
#endif
    get_graph() = Graph(get_map(), get_gg());
    get_hm() = HeuristicMatrix(get_graph());
    get_dhm() = DynamicHeuristicMatrix(get_map(), get_graph());
    get_dhmr() = DHMR(get_graph());
    //get_wmap() = WorkloadMap(get_map(), get_graph());
    get_jg() = JG::JourneyGraph(&env);
    get_operations() = OperationsGenerator().get();
    get_omap() = OperationsMap(get_graph(), get_operations());

    //ASSERT(get_operations_weights().size() == get_operations().size(), "unmatch sizes: " + std::to_string(get_operations_weights().size()) + "!=" + std::to_string(get_operations().size()));

    init_default_planner(env);

    // generate random agents
    /*Randomizer rnd(74124);
    std::ofstream output("agents.txt");
    std::set<uint32_t> S;
    for(int i = 0; i < 6500; i++){
        uint32_t pos = 0;
        while(true){
            pos = rnd.get(1, get_map().get_size() - 1);
            if(get_map().is_free(pos) && !S.count(pos)){
                break;
            }
        }
        S.insert(pos);
    }
    for(uint32_t pos : S){
        pos--;
        output << pos << '\n';
    }
    output.flush();
    std::exit(0);*/
}

void update_environment(SharedEnvironment &env) {
    double task_reveal = env.task_pool.size() * 1.0 / env.num_of_agents;
    // true everywhere
    ASSERT(std::abs(task_reveal - 1.5) < 1e-3, "invalid task reveal");

    // update test type
    {
        if (get_map_type() == MapType::RANDOM) {
            if (env.num_of_agents == 100) {
                get_test_type() = TestType::RANDOM_1;
                // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            } else if (env.num_of_agents == 200) {
                get_test_type() = TestType::RANDOM_2;
                // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            } else if (env.num_of_agents == 400) {
                get_test_type() = TestType::RANDOM_3;
                // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            } else if (env.num_of_agents == 700) {
                get_test_type() = TestType::RANDOM_4;
                // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            } else if (env.num_of_agents == 800) {
                get_test_type() = TestType::RANDOM_5;
                // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            } else {
                // FAILED_ASSERT("invalid test");
            }
        } else if (get_map_type() == MapType::CITY) {
            // FAILED_ASSERT("TODO");
            // CITY-02:
            // 3000 failed
            // 3500 ok
            // ASSERT(env.num_of_agents < 3500, "invalid num of agents");
        } else if (get_map_type() == MapType::GAME) {
            get_test_type() = TestType::GAME;
            // timesteps = 5000
            // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            // agents = 6500
            // ASSERT(env.num_of_agents == get_test_info().steps_num, "invalid num of agents");
        } else if (get_map_type() == MapType::WAREHOUSE) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::WAREHOUSE;
                // timesteps = 5000
                // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            } else {
                // FAILED_ASSERT("invalid test");
            }
        } else if (get_map_type() == MapType::SORTATION) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::SORTATION;
                // ASSERT(env.curr_timestep < get_test_info().steps_num, "invalid timestep");
            } else {
                // FAILED_ASSERT("invalid test");
            }
        } else {
            FAILED_ASSERT("invalid map");
        }
    }

    /*std::set<int> tasks_size;
    for(auto &[t, task] : env.task_pool) {
        tasks_size.insert(static_cast<int>(task.locations.size()));
    }

    if(get_map_type() == MapType::RANDOM) {
        ASSERT(tasks_size.size() <= 5, "invalid task size");
    } else if(get_map_type() == MapType::GAME) {
        // OK
        ASSERT(tasks_size.size() == 2, "invalid task size");

        // failed
        auto s = std::set<int>{1, 2};
        ASSERT(tasks_size == s, "invalid task size");
    } else{
        ASSERT(tasks_size == std::set<int>{2}, "invalid task size");
    }

    return;*/

    if (get_robots_handler().size() == 0) {
        get_robots_handler() = RobotsHandler(env.num_of_agents);
    }
    get_robots_handler().update(env);

    static int prev_timestep_updated = -1;
    if (prev_timestep_updated == env.curr_timestep) {
        // for planner
        get_dhmr().update(env.curr_timestep, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
        return;
    }
    prev_timestep_updated = env.curr_timestep;
    get_dhm().update(env, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
    //get_dhmr().update(env.curr_timestep, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
    //get_wmap().update(env, get_now());
}

#ifdef ENABLE_SCHEDULER_TRICK
std::vector<Action> &get_myplan() {
    static std::vector<Action> plan;
    return plan;
}
#endif
