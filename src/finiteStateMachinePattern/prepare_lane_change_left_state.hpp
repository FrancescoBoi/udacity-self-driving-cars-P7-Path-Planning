#pragma once
#ifndef PREPARE_LANE_CHANGE_LEFT_STATE_H
#define PREPARE_LANE_CHANGE_LEFT_STATE_H
#include "vehicle_sate.hpp"

namespace vehicle
{
    class PrepLaneChangeLeftState : public VehicleState
    {
    public:
        std::string get_state_name(VehicleState*);
        VehicleState* get_next_best_state(VehicleState*);
        void calc_next_best_state(VehicleState*);
        void set_next_state(VehicleState*);
    	static VehicleState& getInstance(); //singleton

    private:
        static const string state_name;
    	PrepLaneChangeLeftState() {};
    	//Keep_lane(const LightOff& other);
    	//Keep_lane& operator=(const LightOff& other);
    };
}
#endif
