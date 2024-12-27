#include <Objects/Basic/assert.hpp>
#include <Objects/Environment/map.hpp>

Map::Map(const SharedEnvironment &env) : rows(env.rows), cols(env.cols) {
    ASSERT(get_size() == env.map.size() + 1, "size mismatch");
    map.resize(env.map.size() + 1);
    map[0] = false;
    for (uint32_t pos = 0; pos < env.map.size(); pos++) {
        map[pos + 1] = env.map[pos] == 0;
    }
}

uint32_t Map::get_rows() const {
    return rows;
}

uint32_t Map::get_cols() const {
    return cols;
}

uint32_t Map::get_size() const {
    return rows * cols + 1;
}

bool Map::is_free(uint32_t pos) const {
    ASSERT(pos < map.size(), "invalid pos");
    return map[pos];
}

Map &get_map() {
    static Map map;
    return map;
}
