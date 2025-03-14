#include <Planner/epibt_lns.hpp>

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

EPIBT_LNS::RetType EPIBT_LNS::try_build(uint32_t r, uint32_t &counter, uint32_t depth) {
    if (counter > 1000 || (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return RetType::FAILED;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
                return RetType::ACCEPTED;
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return RetType::REJECTED;
            }
        } else if (to_r != -2 && visited[to_r] != visited_counter) {
            if (rnd.get_d() < 0.2) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");
            ASSERT(visited[to_r] != visited_counter, "already visited");

            remove_path(to_r);
            add_path(r);

            RetType res = try_build(to_r, ++counter, depth + 1);
            if (res == RetType::ACCEPTED) {
                return res;
            } else if (res == RetType::REJECTED) {
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
    return RetType::FAILED;
}

bool EPIBT_LNS::try_build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    RetType res = try_build(r, counter, 0);
    if (res != RetType::ACCEPTED) {
        add_path(r);
        return false;
    }
    return true;
}

EPIBT_LNS::RetType EPIBT_LNS::build(uint32_t r, uint32_t depth, uint32_t &counter) {
    if (counter == -1 || (counter % 16 == 0 && get_now() >= end_time)) {
        counter = -1;
        return RetType::FAILED;
    }

    visited[r] = visited_counter;
    uint32_t old_desired = desires[r];

    for (uint32_t desired: robot_desires[r]) {
        desires[r] = desired;
        uint32_t to_r = get_used(r);
        if (to_r == -1) {
            add_path(r);
            if (consider()) {
                return RetType::ACCEPTED;
            } else {
                remove_path(r);
                desires[r] = old_desired;
                return RetType::REJECTED;
            }
        } else if (to_r != -2) {
            if (counter > 3000 && depth >= 6) {
                continue;
            }

            ASSERT(0 <= to_r && to_r < robots.size(), "invalid to_r");

            if (desires[to_r] != 0 && rnd.get_d() < 0.8) {
                continue;
            }

            remove_path(to_r);
            add_path(r);

            RetType res = build(to_r, depth + 1, ++counter);
            if (res == RetType::ACCEPTED) {
                return res;
            } else if (res == RetType::REJECTED) {
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
    return RetType::FAILED;
}

bool EPIBT_LNS::build(uint32_t r) {
    ++visited_counter;
    old_score = cur_score;
    remove_path(r);
    uint32_t counter = 0;
    RetType res = build(r, 0, counter);
    if (res != RetType::ACCEPTED) {
        add_path(r);
        return false;
    }
    return true;
}

EPIBT_LNS::EPIBT_LNS(const std::vector<Robot> &robots, TimePoint end_time)
    : EPIBT(robots, end_time), visited(robots.size()) {
}

void EPIBT_LNS::solve(uint64_t seed) {
    rnd = Randomizer(seed);

    temp = 0;
    for (uint32_t r: order) {
        if (get_now() >= end_time) {
            break;
        }
        pibt_step++;
        if (desires[r] != 0) {
            continue;
        }
        build(r);
    }

    temp = 0.001;

    for (; get_now() < end_time; pibt_step++) {
        uint32_t r = rnd.get(0, robots.size() - 1);
        try_build(r);
        temp *= 0.999;
    }
}

uint32_t EPIBT_LNS::get_step() const {
    return pibt_step;
}
