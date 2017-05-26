//
// Created by WindnBike on 2017/5/26.
//

#include "teleop_vehicle_joy/tele_vehicle.h"

namespace teleop_vehicle_joy {

    int TeleVehicle::changeVal(Phy_val &phy_val, const double &vc) {
        double val;
        if (vc == 0) {
            TeleVehicle::restoreVal(phy_val, phy_val.restore_val);
        } else if (vc > 0) {
            val = phy_val.value + phy_val.max_1st_dif * vc;
            if (val < phy_val.max_val) phy_val.value = val;
            else phy_val.value = phy_val.max_val;
        } else if (vc < 0) {
            val = phy_val.value + phy_val.max_1st_rdif * vc;
            if (val > phy_val.max_rval) phy_val.value = val;
            else phy_val.value = phy_val.max_rval;
        }
        return 0;
    }

    int TeleVehicle::restoreVal(Phy_val &phy_val, const double &rv) {
        if (phy_val.value > rv) {
            if (phy_val.value > rv + phy_val.max_1st_dif) phy_val.value -= phy_val.max_1st_dif;
            else phy_val.value = rv;
        } else if (phy_val.value < rv) {
            if (phy_val.value < rv + phy_val.max_1st_rdif) phy_val.value -= phy_val.max_1st_rdif;
            else phy_val.value = rv;
        }
        return 0;
    }

    int TeleVehicle::changeVel(const double &vc) {
        if (!changeVal(velocity, vc)) return 1;
        return 0;
    }

    int TeleVehicle::changeAng(const double &vc) {
        if (!changeVal(angular, vc)) return 1;
        return 0;
    }

    int TeleVehicle::restoreVel() {
        if (!restoreVal(velocity, velocity.restore_val)) return 1;
        return 0;
    }

    int TeleVehicle::restoreAng() {
        if (!restoreVal(angular, angular.restore_val)) return 1;
        return 0;
    }

    int TeleVehicle::restore(bool vel, bool ang) {
        if (vel) restoreVel();
        if (ang) restoreAng();
        return 0;
    }

    int TeleVehicle::reset() {
        restoreVal(velocity, 0);
        restoreVal(angular, 0);;
        return 0;
    }
}  // namespace teleop_vehicle_joy