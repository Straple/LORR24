#include <Objects/Environment/environment.hpp>

#include <Scheduler/journey_graph.hpp>
#include <Scheduler/scheduler.hpp>
#include <Tools/tools.hpp>
#include <settings.hpp>

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

    return;

    get_map() = Map(env);
    get_guidance_map() = GuidanceMap(get_map_type(), get_map());
#ifdef ENABLE_GG_SOLVER
    std::ifstream input("Tmp/gg" + std::to_string(get_unique_id()));
    input >> get_gg();
#else
    Printer().get() = std::ofstream("printer.txt");
    // warehouse bad guidance map
    if (get_map_type() == MapType::RANDOM// || get_map_type() == MapType::WAREHOUSE
            ) {
        get_gg() = GraphGuidance(get_guidance_map());
    } else {
        get_gg() = GraphGuidance(env);
    }
#endif
    get_graph() = Graph(get_map(), get_gg());
    get_hm() = HeuristicMatrix(get_graph());
    get_dhm() = DynamicHeuristicMatrix(get_map(), get_graph());
    get_jg() = JG::JourneyGraph(&env);
    get_operations() = OperationsGenerator().get();
    get_omap() = OperationsMap(get_graph(), get_operations());
    // get_busyness_map() = BusynessMap(get_map());

    // generate random agents
    /*Randomizer rnd(202);
    std::ofstream output("agents.txt");
    std::set<uint32_t> S;
    for(int i = 0; i < 800; i++){
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
    // update test type
    {
        if (get_map_type() == MapType::RANDOM) {
            if (env.num_of_agents == 100) {
                get_test_type() = TestType::RANDOM_1;
                // timesteps = 600
                ASSERT(env.curr_timestep < 600, "invalid timestep");
            } else if (env.num_of_agents == 200) {
                get_test_type() = TestType::RANDOM_2;
                // timesteps = 600
                ASSERT(env.curr_timestep < 600, "invalid timestep");
            } else if (env.num_of_agents == 400) {
                get_test_type() = TestType::RANDOM_3;
                // timesteps = 800
                ASSERT(env.curr_timestep < 800, "invalid timestep");
            } else if (env.num_of_agents == 700) {
                get_test_type() = TestType::RANDOM_4;
                // timesteps = 1000
                ASSERT(env.curr_timestep < 1000, "invalid timestep");
            } else if (env.num_of_agents == 800) {
                get_test_type() = TestType::RANDOM_5;
                // timesteps = 2000
                ASSERT(env.curr_timestep < 2000, "invalid timestep");
            } else {
                FAILED_ASSERT("invalid test");
            }
        } else if (get_map_type() == MapType::CITY) {
            FAILED_ASSERT("kek");
        } else if (get_map_type() == MapType::GAME) {
            // agents = 6500
            ASSERT(env.num_of_agents == 6500, "invalid num of agents");
            get_test_type() = TestType::GAME;

            // 4999 failed
            ASSERT(env.curr_timestep < 5000, "invalid timestep");
        } else if (get_map_type() == MapType::WAREHOUSE) {
            ASSERT(env.curr_timestep < 5000, "invalid timestep");
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::WAREHOUSE;
            } else {
                FAILED_ASSERT("invalid test");
            }
        } else if (get_map_type() == MapType::SORTATION) {
            ASSERT(env.curr_timestep < 5000, "invalid timestep");
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::SORTATION;
            } else {
                FAILED_ASSERT("invalid test");
            }
        } else {
            FAILED_ASSERT("invalid map");
        }
    }

    return;

    get_robots_handler() = RobotsHandler(env);

    static int prev_timestep_updated = -1;
    if (prev_timestep_updated == env.curr_timestep) {
        return;
    }
    prev_timestep_updated = env.curr_timestep;
    get_dhm().update(env, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
}

/*
Порядок выполнения тестов:
WAREHOUSE
...
GAME
CITY-02
CITY-01

кажется, что он идет в обратном порядке от тестов на сайте
*/
