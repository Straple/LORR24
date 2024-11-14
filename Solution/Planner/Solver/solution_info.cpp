#include "solution_info.hpp"

#include "../../Objects/assert.hpp"

bool operator==(const SolutionInfo &lhs, const SolutionInfo &rhs) {
    return lhs.collision_count == rhs.collision_count &&
           lhs.sum_dist_change == rhs.sum_dist_change &&
           lhs.count_forward == rhs.count_forward;
}

bool operator!=(const SolutionInfo &lhs, const SolutionInfo &rhs) {
    return !(lhs == rhs);
}

SolutionInfo operator+(SolutionInfo lhs, const SolutionInfo &rhs) {
    for (uint32_t k = 0; k < PLANNER_DEPTH; k++) {
        lhs.collision_count[k] += rhs.collision_count[k];
        lhs.sum_dist_change[k] += rhs.sum_dist_change[k];
        lhs.count_forward[k] += rhs.count_forward[k];
    }
    return lhs;
}

std::ostream &operator<<(std::ostream &output, const SolutionInfo &info) {
    FAILED_ASSERT("outdated");
    return output;
}
