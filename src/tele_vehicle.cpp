//
// Created by WindnBike on 2017/5/26.
//

#include "teleop_vehicle_joy/tele_vehicle.h"

namespace teleop_vehicle_joy {

    int TeleVehicle::change_vel(const double vc) {
        double vel = vstatus.velocity + vattrib.max_vel_acc * vc;
        if (vc > 0) {
            if (vel < vattrib.max_vel) vstatus.velocity = vel;
            else vstatus.velocity = vattrib.max_vel;
        } else if (vc < 0) {
            if (vel > 0 - vattrib.max_vel) vstatus.velocity = vel;
            else vstatus.velocity = 0 - vattrib.max_vel;
        }
        return 0;
    }
    int TeleVehicle::change_ang(const double ac) {
        double ang = vstatus.angular + vattrib.max_ang_acc * ac;
        if (ac > 0) {
            if (vel < vattrib.max_ang) vstatus.angular = vel;
            else vstatus.angular = vattrib.max_ang;
        } else if (ac < 0) {
            if (vel > 0 - vattrib.max_ang) vstatus.angular = vel;
            else vstatus.angular = 0 - vattrib.max_ang;
        }
        return 0;
    }

    int TeleVehicle::changeVel(){
        if (vstatus.velocity > 0) {
            if (vstatus.velocity > vattrib.max_vel_acc) vstatus.velocity -= vattrib.max_vel_acc;
            else vstatus.velocity = 0;
        } else if (vstatus.velocity < 0) {
            if (vstatus.velocity < 0 - vattrib.max_vel_acc) vstatus.velocity += vattrib.max_vel_acc;
            else vstatus.velocity = 0;
        }
        return 0;
    }

    int TeleVehicle::changeAng(){
        if (vstatus.angular > 0) {
            if (vstatus.angular > vattrib.max_vel_acc) vstatus.angular -= vattrib.max_ang_acc;
            else vstatus.angular = 0;
        } else if (vstatus.angular < 0) {
            if (vstatus.angular < 0 - vattrib.max_vel_acc) vstatus.angular += vattrib.max_ang_acc;
            else vstatus.angular = 0;
        }
        return 0;
    }

    int TeleVehicle::restore(){
        return 0;
    }

    int TeleVehicle::reset(){
        return 0;
    }
}  // namespace teleop_vehicle_joy