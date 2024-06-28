#include "Blimp.h"
/*
 * Init and run calls for level flight mode
 */

// Runs the main level controller
void ModeLevel::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);

    float out_right_com = pilot.y*g.max_man_thr;
    float out_front_com = pilot.x*g.max_man_thr;

    loiter->run_level(out_right_com, out_front_com);
    motors->yaw_out = pilot_yaw*g.max_man_thr;
    motors->down_out = pilot.z*g.max_man_thr;

}
