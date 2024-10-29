#pragma once

#include "SharedEnv.h"

#include "position.hpp"

class DistMachine {

public:
    int get_dist(Position source, Position target, SharedEnvironment *env);
};
