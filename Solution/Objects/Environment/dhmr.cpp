#include <Objects/Environment/dhmr.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/robot_handler.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/operations_map.hpp>

#include <unordered_set>
#include <thread>

uint32_t
DHMR::get_triv(uint32_t r, uint32_t desired, uint32_t robot_node, uint32_t target, std::vector<uint32_t> &visited,
               uint32_t &visited_counter, std::vector<uint32_t> &dp) const {
    visited_counter++;

    constexpr uint32_t MAX_DEPTH = 10;

    uint32_t source = get_omap().get_nodes_path(robot_node, desired).back();
    if (!source) {
        return 0;
    }

    // (dist, node, depth)
    std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>> heap;
    heap.push({get_hm().get(source, target), source, 0});

    uint32_t answer = -1;

    while (!heap.empty()) {
        auto [dist, node, depth] = heap.top();
        heap.pop();

        if (visited[node] == visited_counter) {
            continue;
        }
        visited[node] = visited_counter;

        ASSERT(depth <= MAX_DEPTH, "invalid depth");

        if (depth == MAX_DEPTH || get_graph().get_pos(node).get_pos() == target) {
            if (get_graph().get_pos(node).get_pos() == target) {
                ASSERT(get_hm().get(node, target) == 0, "invalid HM");
            }
            answer = std::min(answer, dist);
            continue;
        }

        dist -= get_hm().get(node, target);

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            ASSERT(0 <= to && to < get_graph().get_nodes_size(), "invalid to");
            if (to && visited[to] != visited_counter) {
                uint32_t to_dist = dist;
                to_dist += get_graph().get_weight(node, action);
                uint32_t to_pos = get_graph().get_pos(to).get_pos();
                uint32_t to_r = pos_to_robot[to_pos];
                if (to_r != -1 && to_r != r) {
                    // + penalty for robot in to
                    //static double workload = matrix.size() * 1.0 / get_map().get_count_free();
                    //static double power = std::max(1.0, workload * 14 * 2 - 2);
                    to_dist += 10;//power;
                }
                to_dist += get_hm().get(to, target);
                heap.push({to_dist, to, depth + 1});
            }
        }
    }
    //std::cout << "cnt: " << cnt << ", ok: " << ok << '\n';
    return answer;
}

uint32_t DHMR::get(uint32_t r, uint32_t desired, uint32_t robot_node, uint32_t target, std::vector<uint32_t> &visited,
                   uint32_t &visited_counter, std::vector<uint32_t> &dp) const {
    visited_counter++;
    constexpr uint32_t MAX_DEPTH = 10;

    uint32_t source = get_omap().get_nodes_path(robot_node, desired).back();
    if (!source) {
        return 0;
    }

    // (dist, node, depth)
    std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>> heap;
    heap.push({get_hm().get(source, target), source, 0});

    while (!heap.empty()) {
        auto [dist, node, depth] = heap.top();
        heap.pop();

        if (visited[node] == visited_counter) {
            continue;
        }
        visited[node] = visited_counter;

        ASSERT(depth <= MAX_DEPTH, "invalid depth");

        if (depth == MAX_DEPTH || get_graph().get_pos(node).get_pos() == target) {
            //uint32_t triv = get_triv(r, desired, robot_node, target);
            //if (triv != dist) {
            //    Printer() << "invalid answers: " + std::to_string(triv) + " != " + std::to_string(dist) << "\n";
            //}
            //ASSERT(triv == dist, "invalid answers: " + std::to_string(triv) + " != " + std::to_string(dist));
            return dist;
        }

        dist -= get_hm().get(node, target);

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);
            ASSERT(0 <= to && to < get_graph().get_nodes_size(), "invalid to");
            if (to && visited[to] != visited_counter) {
                uint32_t to_dist = dist;
                to_dist += get_graph().get_weight(node, action);
                uint32_t to_pos = get_graph().get_pos(to).get_pos();
                uint32_t to_r = pos_to_robot[to_pos];
                if (to_r != -1 && to_r != r) {
                    // + penalty for robot in to
                    //static double workload = matrix.size() * 1.0 / get_map().get_count_free();
                    //static double power = std::max(1.0, workload * 14 * 2 - 2);
                    to_dist += 10;//power;
                }
                to_dist += get_hm().get(to, target);
                heap.push({to_dist, to, depth + 1});
            }
        }
    }

    FAILED_ASSERT("wtf???");
    return 0;
}

void DHMR::update(uint32_t timestep, TimePoint end_time) {
    return;
    Timer timer;
    const auto &robots = get_robots_handler().get_robots();

    pos_to_robot.assign(get_map().get_size(), -1);
    for (uint32_t r = 0; r < robots.size(); r++) {
        uint32_t pos = get_graph().get_pos(robots[r].node).get_pos();
        ASSERT(pos_to_robot[pos] == -1, "invalid pos to robot");
        pos_to_robot[pos] = r;
    }

    {
        matrix.resize(robots.size(), std::vector<uint32_t>(get_operations().size()));
        auto do_work = [&](uint32_t thr) {
            std::vector<uint32_t> visited(get_graph().get_nodes_size());
            uint32_t visited_counter = 1;
            std::vector<uint32_t> dp;
            for (uint32_t r = thr; r < robots.size() && get_now() < end_time && r < DHM_REBUILD_COUNT; r += THREADS) {
                for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
                    matrix[r][desired] = get_triv(r, desired, robots[r].node, robots[r].target, visited,
                                                  visited_counter, dp);
                }
            }
        };

        std::vector<std::thread> threads(THREADS);
        for (uint32_t thr = 0; thr < threads.size(); thr++) {
            threads[thr] = std::thread(do_work, thr);
        }
        for (uint32_t thr = 0; thr < threads.size(); thr++) {
            threads[thr].join();
        }
    }
    /*for (uint32_t r = 0; r < robots.size(); r++) {
        for (uint32_t desired = 0; desired < get_operations().size(); desired++) {
            matrix[r][desired] = get_triv(r, desired, robots[r].node, robots[r].target);
        }
    }*/
#ifdef ENABLE_PRINT_LOG
    Printer() << "DHMR::update: " << timer << '\n';
#endif
}

uint32_t DHMR::get(uint32_t r, uint32_t desired) const {
    ASSERT(r < matrix.size(), "invalid robot");
    ASSERT(matrix[r].size() == get_operations().size(), "invalid matrix");
    ASSERT(0 <= desired && desired < matrix[r].size(), "invalid desired");
    return matrix[r][desired];
}

DHMR &get_dhmr() {
    static DHMR dhmr;
    return dhmr;
}
