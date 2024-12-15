#pragma once

#include <ActionModel.h>

#include <array>
#include <cstdint>
#include <set>
#include <vector>

#include <settings.hpp>

struct Node;
using PNode = Node *;

struct Node {
    PNode parent = nullptr;

    // местоположения роботов в графе
    std::vector<uint32_t> robots;

    // глубина
    uint32_t depth = 0;

    // робота, которого мы сейчас обрабатываем
    uint32_t index = 0;

    // ребра, которые мы использовали
    std::set<uint32_t> used_edges;

    // вершины графа, которые мы использовали
    std::set<uint32_t> used_poses;

    // [depth][robot] = action
    std::array<std::vector<Action>, PLANNER_DEPTH> actions;

    // некоторый скор
    double score = 0;
};
