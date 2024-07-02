#include "Blimp.h"
/*
 * Init and run calls for level flight mode
 */

// Runs the main level controller
void ModeLevel::run()
{
    if (is_zero(blimp.loiter->level_max)){
        //Mode level is the same as manual mode when this parameter is zero, thus warn the user and switch to manual.
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LOIT_LVLMAX is zero. Switching to manual mode.");
        set_mode(Mode::Number::MANUAL, ModeReason::UNAVAILABLE);
    }

    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);

    float out_right_com = pilot.y*g.max_man_thr;
    float out_front_com = pilot.x*g.max_man_thr;

    loiter->run_level(out_right_com, out_front_com);
    motors->yaw_out = pilot_yaw*g.max_man_thr;
    motors->down_out = pilot.z*g.max_man_thr;

}
