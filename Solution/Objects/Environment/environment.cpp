#include <Objects/Environment/environment.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Basic/randomizer.hpp>

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

    Printer().get() = std::ofstream("printer.txt");

    get_map() = Map(env);
    get_guidance_map() = GuidanceMap(get_map_type(), get_map());
    get_gg() = (get_map_type() == MapType::RANDOM ? GraphGuidance(get_guidance_map()) : GraphGuidance(env));
    get_graph() = Graph(get_map(), get_gg());
    get_hm() = HeuristicMatrix(get_graph());
    init_operations();
    get_omap() = OperationsMap(get_graph(), get_operations());

    // generate random agents
    /*Randomizer rnd(2243552234);
    std::ofstream output("agents.txt");
    std::set<uint32_t> S;
    for(int i = 0; i < 9000; i++){
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
    _exit(0);*/
}

void update_environment(SharedEnvironment &env) {
    static int prev_timestep_updated = -1;
    if (prev_timestep_updated == -1) {
        get_robots_handler() = RobotsHandler(env.num_of_agents);
    }
    prev_timestep_updated = env.curr_timestep;
    get_robots_handler().update(env);
}
