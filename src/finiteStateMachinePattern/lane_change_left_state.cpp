#include "lane_change_left_state.hpp"

static vehicle::LaneChangeLeftState& vehicle::LaneChangeLeftState::getInstance()
{
    static vehicle::LaneChangeLeftState singleton;
    return singleton;
}
