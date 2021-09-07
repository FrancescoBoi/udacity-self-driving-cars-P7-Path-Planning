#include "lane_change_right_state.hpp"

static vehicle::LaneChangeRightState& vehicle::LaneChangeRightState::getInstance()
{
    static vehicle::LaneChangeRightState singleton;
    return singleton;
}
