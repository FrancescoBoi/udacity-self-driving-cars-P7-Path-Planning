#pragma once
#ifndef KEEP_LANE_STATE_H
#define KEEP_LANE_STATE_H
#include "vehicle_state.hpp"
#include<map>
#include<vector>
namespace vehicle
{
    class KeepLaneState : public VehicleState
    {
    public:
        std::string get_state_name(VehicleState*);
        VehicleState* get_next_best_state(VehicleState*);
        void calc_next_best_state(VehicleState*);
        void set_next_state(VehicleState*);
    	static KeepLaneState& getInstance(); //singleton

    private:
        static const string state_name;
        static const std::vector<vehicle_states> possible_next_states;
        typedef int (KeepLaneState::*MFP)(int);
        std::map <vehicle_states, MFP> transition_map;
        KeepLaneState() {
            transition_map.insert( std::make_pair(keep_lane, &KeepLaneState::keepLaneCost ));
            transition_map.insert( std::make_pair(prepare_lane_change_left,
                &KeepLaneState::PrepareLaneChangeLeft));
            transition_map.insert( std::make_pair(prepare_lane_change_right,
                &KeepLaneState::PrepareLaneChangeRight));
        };
        Keep_lane(const Keep_lane& other);
    	Keep_lane& operator=(const Keep_lane& other);
    };
}
#endif
