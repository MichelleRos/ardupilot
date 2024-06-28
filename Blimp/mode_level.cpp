#include "Blimp.h"
/*
 * Init and run calls for level flight mode
 */

// Runs the main manual controller
void ModeLevel::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);
    motors->right_out = pilot.y*g.max_man_thr;
    motors->front_out = pilot.x*g.max_man_thr;
    motors->yaw_out = pilot_yaw*g.max_man_thr;
    motors->down_out = pilot.z*g.max_man_thr;
}
