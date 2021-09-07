#pragma once
#ifndef PREPARE_LANE_CHANGE_RIGHT_STATE_H
#define PREPARE_LANE_CHANGE_RIGHT_STATE_H
#include "vehicle_sate.hpp"

namespace vehicle
{
    class PrepLaneChangeRightState : public VehicleState
    {
    public:
        std::string get_state_name(VehicleState*);
        VehicleState* get_next_best_state(VehicleState*);
        void calc_next_best_state(VehicleState*);
        void set_next_state(VehicleState*);
    	static VehicleState& getInstance(); //singleton

    private:
        static const string state_name;
    	//Keep_lane(const LightOff& other);
    	//Keep_lane& operator=(const LightOff& other);
    };
}
#endif
