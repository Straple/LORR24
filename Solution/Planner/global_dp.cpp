#include "global_dp.hpp"

#include "environment.hpp"
#include "../assert.hpp"
#include "planner_solver.hpp"

GlobalDP &get_global_dp() {
    static GlobalDP dp;
    return dp;
}

int GlobalDP::get_robot(int pos) const {
    return pos_to_robot[pos];
}

void GlobalDP::change_map_robots_cnt(int d, int pos, int val, SolutionInfo &info) {
    info.collision_count -= map_robots_cnt[d][pos] * (map_robots_cnt[d][pos] - 1);
    map_robots_cnt[d][pos] += val;
    info.collision_count += map_robots_cnt[d][pos] * (map_robots_cnt[d][pos] - 1);
}

void GlobalDP::change_map_edge_robots_cnt(int d, int pos, int to, int val, SolutionInfo &info) {
    if (pos > to) {
        std::swap(pos, to);
    }
    if (to - pos == 1) {
        // gor
        info.collision_count -= map_edge_robots_cnt_gor[d][pos] * (map_edge_robots_cnt_gor[d][pos] - 1);
        map_edge_robots_cnt_gor[d][pos] += val;
        info.collision_count += map_edge_robots_cnt_gor[d][pos] * (map_edge_robots_cnt_gor[d][pos] - 1);
    } else {
        // ver
        ASSERT(to - pos == get_env().get_cols(), "invalid pos and to");

        info.collision_count -= map_edge_robots_cnt_ver[d][pos] * (map_edge_robots_cnt_ver[d][pos] - 1);
        map_edge_robots_cnt_ver[d][pos] += val;
        info.collision_count += map_edge_robots_cnt_ver[d][pos] * (map_edge_robots_cnt_ver[d][pos] - 1);
    }
}

void GlobalDP::init(SharedEnvironment *env) {
    map_robots_cnt.assign(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size(), 0));
    map_edge_robots_cnt_ver.assign(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size(), 0));
    map_edge_robots_cnt_gor.assign(PLANNER_DEPTH, std::vector<uint32_t>(get_env().get_size(), 0));

    pos_to_robot.assign(get_env().get_size(), -1);
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        int pos = env->curr_states[r].location;
        ASSERT(pos_to_robot[pos] == -1, "pos_to_robot already init by other robot");
        pos_to_robot[pos] = r;
    }
}
