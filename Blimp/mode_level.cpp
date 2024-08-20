#include "Blimp.h"
/*
 * Init and run calls for level flight mode
 */

bool ModeLevel::init(bool ignore_checks)
{
    if (is_zero(blimp.loiter->level_max)){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LOIT_LVLMAX is zero. Leveling is disabled.");
    }
    if (is_zero(blimp.loiter->max_vel_zs)){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LOIT_MAX_VEL_ZS is zero. Z rate stabilization is manual.");
    }
    if (is_zero(blimp.loiter->max_vel_yaws)){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LOIT_MAX_VEL_YAWS is zero. Yaw rate stabilization is manual.");
    }
    return true;
}

// Runs the main level controller
void ModeLevel::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);

    float out_right_com = pilot.y*g.max_man_thr;
    float out_front_com = pilot.x*g.max_man_thr;
    float out_down_com = pilot.z*g.max_man_thr;
    float out_yaw_com = pilot_yaw*g.max_man_thr;

    loiter->run_level_roll(out_right_com);
    loiter->run_level_pitch(out_front_com);
    loiter->run_down_stab(out_down_com);
    loiter->run_yaw_stab(out_yaw_com);

}
