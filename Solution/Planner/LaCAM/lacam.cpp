#include <Planner/LaCAM/lacam.hpp>

#include <Planner/LaCAM/node.hpp>

std::vector<Action> lacam_solve() {
    struct Comparator {
        bool operator()(const PNode lhs, const PNode rhs) const {
            return lhs->score > rhs->score;
        }
    };
    std::multiset<PNode, Comparator> S;

    // add start
    {
        auto start = new Node();
        for (uint32_t depth = 0; depth < PLANNER_DEPTH; depth++) {
            start->actions[depth].resize(get_robots_handler().size());
        }
        for (uint32_t r = 0; r < get_robots_handler().size(); r++) {
            start->robots.push_back(get_robots_handler().get_robot(r).node);
        }
        S.insert(start);
    }

    // garbage collector
    std::vector<PNode> gc;

    std::vector<Action> answer;

    while (!S.empty()) {
        auto node = *S.begin();
        S.erase(S.begin());
        //gc.push_back(node);

        std::cout << S.size() << ' ' << node->depth << ' ' << node->index << std::endl;

        ASSERT(node->depth <= PLANNER_DEPTH, "invalid depth");
        ASSERT(node->index < get_robots_handler().size(), "invalid index");

        if (node->depth == PLANNER_DEPTH) {
            std::cout << "FIND!" << std::endl;
            answer = node->actions[0];
            break;
        }

        for (uint32_t action = 0; action < 3; action++) {
            uint32_t pos = node->robots[node->index];
            uint32_t to_edge = get_graph().get_to_edge(pos, action);
            uint32_t to_node = get_graph().get_to_node(pos, action);
            // проверяем, что не коллизим
            if (!to_edge || !to_node || node->used_edges.count(to_edge)) {
                continue;
            }
            uint32_t to_pos = get_graph().get_pos(to_node).get_pos();
            if (node->used_poses.count(to_pos)) {
                continue;
            }

            auto to = new Node(*node);
            to->parent = node;
            to->index++;
            to->used_edges.insert(to_edge);
            to->used_poses.insert(to_pos);
            to->actions[node->depth][node->index] = static_cast<Action>(action);

            to->score++;// += to->index == get_robots_handler().size();

            if (to->index == get_robots_handler().size()) {
                // flush
                to->depth++;
                to->index = 0;
                to->used_edges.clear();
                to->used_poses.clear();
            }

            S.insert(to);
        }

        delete node;
    }

    while (!S.empty()) {
        auto node = *S.begin();
        S.erase(S.begin());
        gc.push_back(node);
    }

    for (auto node: gc) {
        delete node;
    }

    return answer;
}
