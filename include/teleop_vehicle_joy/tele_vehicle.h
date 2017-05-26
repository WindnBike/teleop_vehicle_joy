//
// Created by WindnBike on 2017/5/26.
//

#ifndef TELEOP_VEHICLE_JOY_TELE_VEHICLE_H
#define TELEOP_VEHICLE_JOY_TELE_VEHICLE_H

namespace teleop_vehicle_joy {

    struct Phy_val {
        double max_val;
        double max_rval;
        double max_1st_dif;
        double max_1st_rdif;
        double res_1st_dif;
        double restore_val = 0;
        double value = 0;
    };

    class TeleVehicle {
    public:

        TeleVehicle(const Phy_val &vel, const Phy_val &ang) : velocity(vel), angular(ang) {}

        int changeVel(const double& vc);
        int changeAng(const double& vc);
        int restoreVel();
        int restoreAng();
        int reset();

    private:
        int changeVal(Phy_val &phy_val, const double& vc);
        int restoreVal(Phy_val &phy_val, const double& rv);
        Phy_val velocity, angular;
    };

}  // namespace teleop_vehicle_joy

#endif //TELEOP_VEHICLE_JOY_TELE_VEHICLE_H
