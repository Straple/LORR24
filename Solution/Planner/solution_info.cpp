#include "solution_info.hpp"

bool operator==(const SolutionInfo &lhs, const SolutionInfo &rhs) {
    return lhs.collision_count == rhs.collision_count &&
           lhs.sum_dist_change == rhs.sum_dist_change &&
           lhs.count_forward == rhs.count_forward;
}

bool operator!=(const SolutionInfo &lhs, const SolutionInfo &rhs) {
    return !(lhs == rhs);
}

std::ostream &operator<<(std::ostream &output, const SolutionInfo &info) {
    return output << info.collision_count << ' ' << info.sum_dist_change << ' ' << info.count_forward;
}
