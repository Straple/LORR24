#include <Objects/Environment/dynamic_heuristic_matrix.hpp>

#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <settings.hpp>

#include <atomic>
#include <thread>

void DynamicHeuristicMatrix::rebuild(uint32_t source, const std::unordered_set<uint32_t> &robots_pos) {
    ASSERT(source < matrix.size(), "invalid source");
    auto &dists = matrix[source];
    dists.assign(get_graph().get_nodes_size(), -1);

    // (dist, node)
    std::priority_queue<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>, std::greater<>> heap;
    std::vector<bool> visited(dists.size());

    for (uint32_t dir = 0; dir < 4; dir++) {
        ASSERT(Position(source, dir).is_valid(), "invalid");
        heap.push({0, get_graph().get_node(Position(source, dir))});
    }

    while (!heap.empty()) {
        auto [dist, node] = heap.top();
        heap.pop();

        ASSERT(0 < node && node < get_graph().get_nodes_size(), "invalid node");
        ASSERT(node < visited.size(), "invalid node");

        if (visited[node]) {
            continue;
        }
        visited[node] = true;

        uint32_t inv = get_graph().get_node(get_graph().get_pos(node).rotate().rotate());
        dists[inv] = dist;

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            ASSERT(0 <= to && to < get_graph().get_nodes_size(), "invalid to");
            if (to && !visited[to]) {
                uint32_t to_inv = get_graph().get_node(get_graph().get_pos(to).rotate().rotate());
                uint64_t to_dist = dist + 1 + robots_pos.count(get_graph().get_pos(to).get_pos());
                if (dists[to_inv] > to_dist) {
                    dists[to_inv] = to_dist;
                    heap.push({to_dist, to});
                }
            }
        }
    }
}

DynamicHeuristicMatrix::DynamicHeuristicMatrix(const Map &map) {
    matrix.resize(map.get_size());
    timestep_updated.resize(map.get_size());
    for (uint32_t pos = 1; pos < matrix.size(); pos++) {
        if (Position(pos, 0).is_valid()) {
            matrix[pos].assign(get_graph().get_nodes_size(), -1);
        }
    }
}

void DynamicHeuristicMatrix::update(SharedEnvironment &env, TimePoint end_time) {
#ifdef ENABLE_DHM
    Timer timer;

    std::unordered_set<uint32_t> robots_pos;
    for (auto robot: get_robots_handler().get_robots()) {
        ASSERT(0 < robot.node && robot.node < get_graph().get_nodes_size(), "invalid node");
        robots_pos.insert(get_graph().get_pos(robot.node).get_pos());
    }

    // (timestep updated, target pos)
    std::vector<std::pair<uint32_t, uint32_t>> pool;

    for (auto &[t, task]: env.task_pool) {
        uint32_t target = task.get_next_loc() + 1;
        ASSERT(0 < target && target < get_map().get_size(), "invalid target");
        pool.emplace_back(timestep_updated[target], target);
    }
    std::sort(pool.begin(), pool.end());

    std::atomic<uint32_t> total_rebuild = 0;

    auto do_work = [&](uint32_t thr) {
        for (uint32_t index = thr; index < pool.size() && get_now() < end_time; index += THREADS) {
            auto [_, target] = pool[index];

            timestep_updated[target] = env.curr_timestep;
            rebuild(target, robots_pos);
            ++total_rebuild;
        }
    };

    std::vector<std::thread> threads(THREADS);
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr] = std::thread(do_work, thr);
    }
    for (uint32_t thr = 0; thr < THREADS; thr++) {
        threads[thr].join();
    }

#ifdef ENABLE_PRINT_LOG
    Printer() << "total rebuild: " << total_rebuild << '\n';
    Printer() << "rebuild time: " << timer << '\n';
#endif

#endif
}

uint32_t DynamicHeuristicMatrix::get(uint32_t source, uint32_t target) const {
    if(!target){
        return INVALID_DIST;
    }

#ifdef ENABLE_DHM
    ASSERT(target < matrix.size(), "invalid target");

    if (!matrix[target].empty()) {
        ASSERT(source < matrix[target].size(), "invalid source");
        return matrix[target][source];
    } else {
        return get_hm().get_to_pos(source, target);
    }
#else
    return get_hm().get_to_pos(source, target);
#endif
}

DynamicHeuristicMatrix &get_dhm() {
    static DynamicHeuristicMatrix dhm;
    return dhm;
}
