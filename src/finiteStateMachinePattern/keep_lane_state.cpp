#include "keep_lane_state.hpp"

const KeepLaneState::state_name = "keep_lane";
const KeepLaneState::std::vector<vehicle_states> possible_next_states = {keep_lane, prepare_lane_change_left,
    prepare_lane_change_right};

static vehicle::KeepLaneState& vehicle::KeepLaneState::getInstance()
{
    static vehicle::KeepLaneState singleton;
    return singleton;
}
