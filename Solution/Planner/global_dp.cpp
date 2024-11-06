#include "global_dp.hpp"

#include "../Objects/environment.hpp"
#include "planner_solver.hpp"

#include "../Objects/assert.hpp"
#include "../Objects/dsu.hpp"

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

std::vector<std::vector<int>> GlobalDP::split_robots(SharedEnvironment *env) {
    DSU dsu(env->num_of_agents);

    std::vector<int> map(env->map.size(), -1);

    auto paint = [&](Position source, uint32_t r) {
        std::vector<Position> Q0, Q1;
        std::set<Position> visited;

        Q0.push_back(source);
        visited.insert(source);

        int d = 0;
        while (!Q0.empty() || !Q1.empty()) {
            if (Q0.empty()) {
                std::swap(Q0, Q1);
                d++;
            }

            if (d > PLANNER_DEPTH) {
                break;
            }

            Position p = Q0.back();
            Q0.pop_back();

            ASSERT(p.is_valid(), "p is invalid");

            // paint
            {
                if (map[p.pos] == -1) {
                    map[p.pos] = r;
                } else {
                    dsu.uni(map[p.pos], r);
                }
            }

#define STEP(init)                                                  \
    {                                                               \
        Position to = (init);                                       \
        if (to.is_valid() && !visited.count(to)) {                  \
            visited.insert(to);                                     \
            Q1.push_back(to);                                       \
        }                                                           \
    }

            STEP(p.move_forward());
            STEP(p.rotate());
            STEP(p.counter_rotate());

#undef STEP
        }
    };

    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        paint(Position(env->curr_states[r].location, env->curr_states[r].orientation), r);
    }

    std::vector<std::vector<int>> ans(env->num_of_agents);
    for (uint32_t r = 0; r < env->num_of_agents; r++) {
        ans[dsu.get(r)].push_back(r);
    }
    for (uint32_t r = 0; r < ans.size(); r++) {
        if (ans[r].empty()) {
            std::swap(ans[r], ans.back());
            ans.pop_back();
            r--;
        }
    }
    //std::cout << "Total groups: " << ans.size() << std::endl;
    /*for(auto group : ans){
        std::cout << group.size() << ": ";
        for(int r : group){
            std::cout << r << ' ';
        }
        std::cout << std::endl;
    }*/
    return ans;
}

