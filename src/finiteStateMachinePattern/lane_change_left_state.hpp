#pragma once
#ifndef LANE_CHANGE_LEFT_STATE_H
#define LANE_CHANGE_LEFT_STATE_H
#include "vehicle_sate.hpp"

namespace vehicle
{
    class LaneChangeLeftState : public VehicleState
    {
    public:
        std::string get_state_name(VehicleState*);
        VehicleState* get_next_best_state(VehicleState*);
        void calc_next_best_state(VehicleState*);
        void set_next_state(VehicleState*);
    	static VehicleState& getInstance(); //singleton

    private:
        static const string state_name;
    	LaneChangeLeftState() {};
    	//Keep_lane(const LightOff& other);
    	//Keep_lane& operator=(const LightOff& other);
    };
}
#endif
