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

    get_map() = Map(env);
    get_guidance_map() = GuidanceMap(get_map_type(), get_map());
#ifdef ENABLE_GG_SOLVER
    std::ifstream input("Tmp/gg" + std::to_string(get_unique_id()));
    input >> get_gg();
#else
    Printer().get() = std::ofstream("printer");
    //get_gg() = GraphGuidance(env);
    get_gg() = GraphGuidance(get_guidance_map());
#endif
    get_graph() = Graph(get_map(), get_gg());
    get_hm() = HeuristicMatrix(get_graph());
    get_dhm() = DynamicHeuristicMatrix(get_map(), get_graph());
    get_jg() = JG::JourneyGraph(&env);
    get_operations() = OperationsGenerator().get();
    get_omap() = OperationsMap(get_graph(), get_operations());
    // get_busyness_map() = BusynessMap(get_map());

    // generate random agents
    /*Randomizer rnd(5340000);
    std::ofstream output("agents.txt");
    std::set<uint32_t> S;
    for(int i = 0; i < 600; i++){
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
    get_robots_handler() = RobotsHandler(env);

    static int prev_timestep_updated = -1;
    if (prev_timestep_updated == env.curr_timestep) {
        return;
    }
    prev_timestep_updated = env.curr_timestep;
    get_dhm().update(env, get_now() + Milliseconds(DHM_REBUILD_TIMELIMIT));
}
