#include <Planner/PIBT/epibt_lns.hpp>

#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/environment.hpp>
#include <settings.hpp>

bool EPIBT_LNS::consider() {
    return old_score - 1e-6 <= cur_score
// old_score > cur_score
#ifdef ENABLE_LNS_ANNEALING
           || rnd.get_d() < std::exp(-((old_score - cur_score) / old_score) / temp)
#endif
            ;
}

uint32_t EPIBT_LNS::try_build(uint32_t r, uint32_t &counter, uint32_t depth) {
    if (counter > 1000 || (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else if (to_r != -2 && visited[to_r] != visited_counter) {
            if (rnd.get_d() < 0.2) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            ASSERT(visited[to_r] != visited_counter, "already visited");

            remove_path(to_r);
            add_path(r);

            uint32_t res = try_build(to_r, ++counter, depth + 1);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                add_path(to_r);
                desires[r] = old_desired;
                return res;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    desires[r] = old_desired;
    visited[r] = 0;
    return 0;
}

bool EPIBT_LNS::try_build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = try_build(r, counter, 0);
    if (res == 0 || res == 2) {
        add_path(r);
        return false;
    }
    return true;
}

uint32_t EPIBT_LNS::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 || (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return 2;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
                return 1;// accepted
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return 2;// not accepted
            }
        } else if (to_r != -2) {
            if (counter > 3000 && depth >= 6) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");

            if (desires[to_r] != 0
#ifdef ENABLE_EPIBT_LNS_TRICK
                && rnd.get_d() < 0.8
#endif
            ) {
                continue;
            }

            remove_path(to_r);
            add_path(r);

            uint32_t res = build(to_r, depth + 1, ++counter);
            if (res == 1) {
                return res;
            } else if (res == 2) {
                remove_path(r);
                add_path(to_r);
                desires[r] = old_desired;
                return res;
            }

            remove_path(r);
            add_path(to_r);
        }
    }

    visited[r] = 0;
    desires[r] = old_desired;
    return 0;
}

bool EPIBT_LNS::build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    uint32_t res = build(r, 0, counter);
    if (res == 0 || res == 2) {
        add_path(r);
        return false;
    }
    return true;
}

EPIBT_LNS::EPIBT_LNS(const std::vector<Robot> &robots, TimePoint end_time)
    : EPIBT(robots, end_time), visited(robots.size()), best_desires(robots.size()) {
}

void EPIBT_LNS::solve(uint64_t seed) {
    rnd = Randomizer(seed);

    temp = 0;
    for (uint32_t r: order) {
        if (get_now() >= end_time) {
            break;
        }
        if (desires[r] != 0) {
            continue;
        }
        build(r);
    }

    best_desires = desires;
    best_score = cur_score;

    temp = 0.001;

    for (pibt_step = 0; get_now() < end_time; pibt_step++) {
        uint32_t r = rnd.get(0, robots.size() - 1);
        try_build(r);
        temp *= 0.999;
    }
    if (best_score + 1e-6 < cur_score) {
        best_desires = desires;
        best_score = cur_score;
    }
}

std::vector<Action> EPIBT_LNS::get_actions() const {
    std::vector<Action> answer(robots.size());
    for (uint32_t r = 0; r < robots.size(); r++) {
        answer[r] = get_operations()[best_desires[r]][0];
        if (best_desires[r] == 0) {
            auto dist = std::min({get_hm().get(get_graph().get_to_node(robots[r].node, 1), robots[r].target),
                                  get_hm().get(get_graph().get_to_node(robots[r].node, 2), robots[r].target),
                                  get_hm().get(get_graph().get_to_node(robots[r].node, 3), robots[r].target)});
            if (dist == get_hm().get(get_graph().get_to_node(robots[r].node, 1), robots[r].target)) {
                answer[r] = Action::CR;
            } else if (dist == get_hm().get(get_graph().get_to_node(robots[r].node, 2), robots[r].target)) {
                answer[r] = Action::CCR;
            } else {
                answer[r] = Action::W;
            }
        }
    }
    return answer;
}

double EPIBT_LNS::get_score() const {
    return best_score;
}

uint32_t EPIBT_LNS::get_step() const {
    return pibt_step;
}
