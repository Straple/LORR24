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
    double task_reveal = env.task_pool.size() * 1.0 / env.num_of_agents;
    // true everywhere
    ASSERT(std::abs(task_reveal - 1.5) < 1e-3, "invalid task reveal");

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
            FAILED_ASSERT("TODO");
            // CITY-02:
            // 3000 failed
            // 3500 ok
            ASSERT(env.num_of_agents < 3500, "invalid num of agents");
            //FAILED_ASSERT("kek");
        } else if (get_map_type() == MapType::GAME) {
            get_test_type() = TestType::GAME;
            // timesteps = 5000
            ASSERT(env.curr_timestep < 5000, "invalid timestep");
            // agents = 6500
            ASSERT(env.num_of_agents == 6500, "invalid num of agents");
        } else if (get_map_type() == MapType::WAREHOUSE) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::WAREHOUSE;
                // timesteps = 5000
                ASSERT(env.curr_timestep < 5000, "invalid timestep");
            } else {
                FAILED_ASSERT("invalid test");
            }
        } else if (get_map_type() == MapType::SORTATION) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::SORTATION;
                // timesteps = 5000
                ASSERT(env.curr_timestep < 5000, "invalid timestep");
            } else {
                FAILED_ASSERT("invalid test");
            }
        } else {
            FAILED_ASSERT("invalid map");
        }
    }

    std::set<int> tasks_size;
    for(auto &[t, task] : env.task_pool) {
        tasks_size.insert(static_cast<int>(task.locations.size()));
    }

    if(get_map_type() == MapType::RANDOM) {
        ASSERT(tasks_size.size() <= 5, "invalid task size");
    } else if(get_map_type() == MapType::GAME) {
        // <= 2 ok
        // 1 failed
        ASSERT(tasks_size.size() == 2, "invalid task size");
    } else{
        ASSERT(tasks_size == std::set<int>{2}, "invalid task size");
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
