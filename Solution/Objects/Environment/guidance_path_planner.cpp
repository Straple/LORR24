#include <Objects/Environment/guidance_path_planner.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/graph.hpp>
#include <Objects/Environment/heuristic_matrix.hpp>
#include <Objects/Environment/robot_handler.hpp>

#include <set>
#include <thread>
#include <unordered_set>

void GuidancePathPlanner::build(uint32_t r) {
    const uint32_t target = get_robots_handler().get_robot(r).target;

    if (!target) {
        return;
    }

    // (metric, node, depth)
    std::priority_queue<std::tuple<uint32_t, uint32_t, uint32_t>,
                        std::vector<std::tuple<uint32_t, uint32_t, uint32_t>>, std::greater<>>
            heap;

    // parent[to] = from
    std::unordered_map<uint32_t, uint32_t> parent;

    std::unordered_set<uint32_t> visited;

    heap.emplace(0, get_robots_handler().get_robot(r).node, 0);
    parent[get_robots_handler().get_robot(r).node] = -1;
    while (!heap.empty()) {
        auto [metric, node, depth] = heap.top();
        heap.pop();

        ASSERT(node, "invalid node");

        if (visited.count(node)) {
            continue;
        }
        visited.insert(node);


        if (get_graph().get_pos(node).get_pos() == target || depth >= GPP_DEPTH) {
            // TODO: path recovery
            return;
        }

        // F, C, R
        for (uint32_t action = 0; action < 3; action++) {
            uint32_t to = get_graph().get_to_node(node, action);

            if (to && !visited.count(to)) {
                heap.push({metric + get_graph().get_weight(node, action), to, depth + 1});
            }
        }
    }
}

void GuidancePathPlanner::update(uint32_t timestep, TimePoint end_time) {
    ++timestep;
    ETimer timer;
    const auto &robots = get_robots_handler().get_robots();

    pos_to_robot.assign(get_map().get_size(), -1);
    for (uint32_t r = 0; r < robots.size(); r++) {
        uint32_t pos = get_graph().get_pos(robots[r].node).get_pos();
        ASSERT(pos_to_robot[pos] == -1, "invalid pos to robot");
        pos_to_robot[pos] = r;
    }

    {
        auto do_work = [&](uint32_t thr) {
            for (uint32_t r = thr; r < robots.size(); r += THREADS) {
                build(r);
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

    PRINT(Printer() << "GuidancePathPlanner::update: " << timer << '\n';);
}