#include <Objects/Environment/dynamic_heuristic_matrix.hpp>

#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <settings.hpp>

#include <atomic>
#include <thread>

void DynamicHeuristicMatrix::rebuild(uint32_t source, uint32_t timestep) {
    ASSERT(source < matrix.size(), "invalid source");
    auto &dists = matrix[source];
    dists.assign(get_graph().get_nodes_size(), -1);

    // (dist, node)
    /*std::priority_queue<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>, std::greater<>> heap;

    for (uint32_t dir = 0; dir < 4; dir++) {
        ASSERT(Position(source, dir).is_valid(), "invalid");
        uint32_t node = get_graph().get_node(Position(source, dir));
        heap.push({0, node});
        dists[node] = 0;
    }

    std::vector<bool> visited(dists.size());

    while (!heap.empty()) {
        auto [d, node] = heap.top();
        heap.pop();

        ASSERT(0 < node && node < get_graph().get_nodes_size(), "invalid node");
        ASSERT(node < visited.size(), "invalid node");

        if (visited[node]) {
            continue;
        }
        visited[node] = true;
        ASSERT(dists[node] == d, "invalid dist");

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            ASSERT(0 <= to && to < get_graph().get_nodes_size(), "invalid to");
            if (to && !visited[to]) {
                uint64_t to_dist = d + 1;
                uint32_t p = get_graph().get_pos(to).get_pos();
                if (action == 0 && used[p] == timestep) {
                    to_dist += weight[p];
                }
                if (dists[to] > to_dist) {
                    dists[to] = to_dist;
                    heap.push({to_dist, to});
                }
            }
        }
    }*/

    // queue[dist - queue_dist] = { node }
    std::vector<std::vector<uint32_t>> queue(1);
    uint32_t queue_dist = 0;

    std::vector<bool> visited(dists.size());

    auto update_queue = [&]() {
        while (!queue.empty() && queue[0].empty()) {
            queue.erase(queue.begin());
            queue_dist++;
        }
    };

    auto push_queue = [&](uint32_t dist, uint32_t node) {
        ASSERT(queue_dist <= dist, "invalid dist");
        uint32_t index = dist - queue_dist;
        if (index >= queue.size()) {
            queue.resize(index + 1);
        }
        queue[index].push_back(node);
    };

    for (uint32_t dir = 0; dir < 4; dir++) {
        ASSERT(Position(source, dir).is_valid(), "invalid");
        uint32_t node = get_graph().get_node(Position(source, dir));
        queue[0].push_back(node);
        dists[node] = 0;
    }

    while (true) {
        update_queue();
        if (queue.empty()) {
            break;
        }
        ASSERT(!queue[0].empty(), "empty");
        uint32_t node = queue[0].back();
        queue[0].pop_back();

        ASSERT(0 < node && node < get_graph().get_nodes_size(), "invalid node");
        ASSERT(node < visited.size(), "invalid node");

        if (visited[node]) {
            continue;
        }
        visited[node] = true;
        ASSERT(dists[node] == queue_dist, "invalid dist");

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            ASSERT(0 <= to && to < get_graph().get_nodes_size(), "invalid to");
            if (to && !visited[to]) {
                uint64_t to_dist = queue_dist + get_graph().get_weight(node, action);
                uint32_t p = get_graph().get_pos(to).get_pos();
                if (action == 0 && used[p] == timestep) {
                    to_dist += weight[p];
                }
                if (dists[to] > to_dist) {
                    dists[to] = to_dist;
                    push_queue(to_dist, to);
                }
            }
        }
    }
}

void DynamicHeuristicMatrix::update_pos(uint32_t pos, int32_t w, uint32_t timestep) {
    if (used[pos] == timestep) {
        weight[pos] += w;
    } else {
        used[pos] = timestep;
        weight[pos] = w;
    }
}

DynamicHeuristicMatrix::DynamicHeuristicMatrix(const Map &map) {
    matrix.resize(map.get_size());
    timestep_updated.resize(map.get_size());
    used.resize(map.get_size(), -1);
    weight.resize(map.get_size(), 0);
    for (uint32_t pos = 1; pos < matrix.size(); pos++) {
        if (Position(pos, 0).is_valid()) {
            //matrix[pos].assign(get_graph().get_nodes_size(), -1);
        }
    }
}

void DynamicHeuristicMatrix::update(SharedEnvironment &env, TimePoint end_time) {
#ifdef ENABLE_DHM
    Timer timer;

    double workload = env.num_of_agents * 1.0 / get_map().get_count_free();

    //call(0): 2540, 12.7575s
    //call(1): 4332, 19.743s
    //call(2): 5194, 28.2415s
    //call(3): 5285, 38.8171s
    //call(4): 4541, 64.0062s
    //call(5): 3594, 131.139s
    //total: 25486
    double power = std::max(1.0, workload * 14 - 1);

    // ACTION WEIGHT = 2

    // workload * 20
    //call(0): 2542, 11.349s
    //call(1): 4332, 19.7347s
    //call(2): 5291, 30.8083s
    //call(3): 4908, 48.2055s
    //call(4): 4328, 73.3719s
    //call(5): 3573, 127.418s
    //total: 24974

    // workload * 21
    //call(0): 2523, 11.2241s
    //call(1): 4320, 18.959s
    //call(2): 5291, 27.6536s
    //call(3): 5285, 37.3194s
    //call(4): 4328, 68.0979s
    //call(5): 3438, 127.48s
    //total: 25185

    // workload * 22
    //call(0): 2525, 16.3123s
    //call(1): 4320, 22.2663s
    //call(2): 5194, 31.0398s
    //call(3): 5285, 39.3911s
    //call(4): 4278, 72.043s
    //call(5): 3699, 112.033s
    //total: 25301

    //workload*23
    //call(0): 2532, 10.7565s
    //call(1): 4320, 21.6136s
    //call(2): 5194, 27.443s
    //call(3): 5007, 45.6257s
    //call(4): 4541, 67.2073s
    //call(5): 3699, 114.081s
    //total: 25293

    //workload*24
    //call(0): 2534, 11.7868s
    //call(1): 4320, 20.1953s
    //call(2): 5194, 29.5569s
    //call(3): 5007, 44.5165s
    //call(4): 4541, 67.0642s
    //call(5): 3603, 126.385s
    //total: 25199

#ifdef ENABLE_PRINT_LOG
    Printer() << "workload: " << workload << '\n';
    Printer() << "power: " << power << '\n';
#endif

    for (auto robot: get_robots_handler().get_robots()) {
        ASSERT(0 < robot.node && robot.node < get_graph().get_nodes_size(), "invalid node");
        Position p = get_graph().get_pos(robot.node);
        update_pos(p.get_pos(), power, env.curr_timestep);
    }

    // (timestep updated, target pos)
    std::vector<std::pair<uint32_t, uint32_t>> pool;

    for (auto &[t, task]: env.task_pool) {
        uint32_t target = task.get_next_loc() + 1;
        ASSERT(0 < target && target < get_map().get_size(), "invalid target");
        pool.emplace_back(timestep_updated[target], target);
    }
    std::sort(pool.begin(), pool.end());
    pool.erase(std::unique(pool.begin(), pool.end()), pool.end());

    std::atomic<uint32_t> total_rebuild = 0;

    auto do_work = [&](uint32_t thr) {
        for (uint32_t index = thr; index < pool.size() && get_now() < end_time; index += THREADS) {
            auto [_, target] = pool[index];

            timestep_updated[target] = env.curr_timestep;
            rebuild(target, env.curr_timestep);
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
    Printer() << "total rebuild: " << total_rebuild << '/' << pool.size() << '\n';
    Printer() << "rebuild time: " << timer << '\n';
#endif

#endif
}

uint32_t DynamicHeuristicMatrix::get(uint32_t source, uint32_t target) const {
    if (!target) {
        return INVALID_DIST;
    }

#ifdef ENABLE_DHM
    ASSERT(target < matrix.size(), "invalid target");

    if (!matrix[target].empty()) {
        source = get_graph().get_to_node(get_graph().get_to_node(source, 1), 1);
        // get_graph().get_node(get_graph().get_pos(source).rotate().rotate());
        ASSERT(source < matrix[target].size(), "invalid source");
        ASSERT(matrix[target][source] != -1, "invalid matrix");
        return matrix[target][source];
    } else {
        return get_hm().get(source, target);
    }
#else
    return get_hm().get(source, target);
#endif
}

DynamicHeuristicMatrix &get_dhm() {
    static DynamicHeuristicMatrix dhm;
    return dhm;
}
