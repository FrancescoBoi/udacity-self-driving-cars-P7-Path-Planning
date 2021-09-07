#include "prepare_lane_change_right_state.hpp"

static vehicle::PrepLaneChangeRightState& vehicle::PrepLaneChangeRightState::getInstance()
{
    static vehicle::PrepLaneChangeRightState singleton;
    return singleton;
}
