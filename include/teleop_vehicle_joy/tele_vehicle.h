//
// Created by WindnBike on 2017/5/26.
//

#ifndef TELEOP_VEHICLE_JOY_TELE_VEHICLE_H
#define TELEOP_VEHICLE_JOY_TELE_VEHICLE_H

namespace teleop_vehicle_joy {

    class TeleVehicle {
    public:
        TeleVehicle(const Attrib &attr) : vattrib(attr) {}

        int change_vel(const double target);

        int change_ang(const double target);

        int changeVel();

        int changeAng();

        int restore();

        int reset();

        struct Attrib {
            double max_vel;
            double max_ang;
            double max_vel_acc;
            double max_ang_acc;
            double restore_vel;
            double restore_ang;
        };

        struct Status {
            double velocity = 0;
            double angular = 0;
            double cruise_vol = 0;
            double cruise_ang = 0;
        };

    private:
        Status vstatus;
        Attrib vattrib;
    };

}  // namespace teleop_vehicle_joy

#endif //TELEOP_VEHICLE_JOY_TELE_VEHICLE_H
