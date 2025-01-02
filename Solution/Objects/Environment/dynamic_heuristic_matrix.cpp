#include <Objects/Environment/dynamic_heuristic_matrix.hpp>

#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <settings.hpp>

#include <atomic>
#include <thread>

void DynamicHeuristicMatrix::rebuild(uint32_t source, const std::unordered_set<uint32_t> &robots_pos) {
    auto &dists = matrix[source];
    dists.assign(get_graph().get_nodes_size(), -1);

    std::vector<bool> visited(get_graph().get_nodes_size());

    // (dist, node)
    std::priority_queue<std::pair<uint64_t, uint32_t>, std::vector<std::pair<uint64_t, uint32_t>>, std::greater<>> heap;

    for (uint32_t dir = 0; dir < 4; dir++) {
        heap.push({0, get_graph().get_node(Position(source, dir))});
    }

    while (!heap.empty()) {
        auto [dist, node] = heap.top();
        heap.pop();

        if (visited[node]) {
            continue;
        }
        visited[node] = true;

        uint32_t inv = get_graph().get_node(get_graph().get_pos(node).rotate().rotate());
        dists[inv] = dist;

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
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
}

void DynamicHeuristicMatrix::update(SharedEnvironment &env, TimePoint end_time) {
    Timer timer;

    std::unordered_set<uint32_t> robots_pos;
    for (auto robot: get_robots_handler().get_robots()) {
        robots_pos.insert(get_graph().get_pos(robot.node).get_pos());
    }

    // (timestep updated, target pos)
    std::vector<std::pair<uint32_t, uint32_t>> pool;

    for (auto &[t, task]: env.task_pool) {
        uint32_t target = task.get_next_loc() + 1;
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
    std::cout << "total rebuild: " << total_rebuild << '\n';
    std::cout << "rebuild time: " << timer << '\n';
#endif
}

uint64_t DynamicHeuristicMatrix::get(uint32_t source, uint32_t target) {
    ASSERT(target < matrix.size(), "invalid target");

    if (!matrix[target].empty()) {
        ASSERT(source < matrix[target].size(), "invalid source");
        return matrix[target][source];
    } else {
        return get_hm().get_to_pos(source, target);
    }
}

DynamicHeuristicMatrix &get_dhm() {
    static DynamicHeuristicMatrix dhm;
    return dhm;
}
