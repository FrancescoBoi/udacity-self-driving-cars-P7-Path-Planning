#pragma once
#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

namespace vehicle
{
    enum vehicle_states {keep_lane, change_lane_left, change_lane_right,
        prepare_lane_change_left, prepare_lane_change_right};

    class VehicleState; // Forward declaration to resolve circular dependency/include

    class VehicleState
    {
    public:
        virtual std::string get_state_name(VehicleState*)=0;
        virtual VehicleState* get_next_best_state(VehicleState*)=0;
        virtual void calc_next_best_state(VehicleState*)=0;
    };
}
#endif  // VEHICLE_STATE_H
