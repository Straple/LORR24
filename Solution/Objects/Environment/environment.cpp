#include <Objects/Environment/environment.hpp>

#include <Scheduler/journey_graph.hpp>
#include <Scheduler/scheduler.hpp>
#include <Tools/tools.hpp>
#include <settings.hpp>

#include <thread>

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
    Printer().get() = std::ofstream("printer.txt");
    get_guidance_map() = GuidanceMap(get_map_type(), get_map());

    if (get_map_type() == MapType::RANDOM) {
        get_gg() = GraphGuidance(get_guidance_map());
    } else {
        get_gg() = GraphGuidance(env);
    }

    get_graph() = Graph(get_map(), get_gg());
    get_hm() = HeuristicMatrix(get_graph());
    init_operations();
    get_omap() = OperationsMap(get_graph(), get_operations());

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
    //ASSERT(std::abs(task_reveal - 1.5) < 1e-3, "invalid task reveal");

    // update test type
    {
        if (get_map_type() == MapType::RANDOM) {
            if (env.num_of_agents == 100) {
                get_test_type() = TestType::RANDOM_1;
            } else if (env.num_of_agents == 200) {
                get_test_type() = TestType::RANDOM_2;
            } else if (env.num_of_agents == 400) {
                get_test_type() = TestType::RANDOM_3;
            } else if (env.num_of_agents == 700) {
                get_test_type() = TestType::RANDOM_4;
            } else if (env.num_of_agents == 800) {
                get_test_type() = TestType::RANDOM_5;
            }
        } else if (get_map_type() == MapType::CITY) {
            // FAILED_ASSERT("TODO");
            // CITY-02:
            // 3000 failed
            // 3500 ok
            // ASSERT(env.num_of_agents < 3500, "invalid num of agents");
        } else if (get_map_type() == MapType::GAME) {
            get_test_type() = TestType::GAME;
        } else if (get_map_type() == MapType::WAREHOUSE) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::WAREHOUSE;
            }
        } else if (get_map_type() == MapType::SORTATION) {
            if (env.num_of_agents == 10'000) {
                get_test_type() = TestType::SORTATION;
            }
        } else {
            FAILED_ASSERT("invalid map");
        }
    }

    static int prev_timestep_updated = -1;
    if (prev_timestep_updated == -1) {
        get_robots_handler() = RobotsHandler(env.num_of_agents);
    }
    get_robots_handler().update(env);
    if (prev_timestep_updated == env.curr_timestep) {
        return;
    }
    prev_timestep_updated = env.curr_timestep;
}
