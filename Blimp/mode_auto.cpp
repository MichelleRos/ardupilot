#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

bool ModeAuto::init(bool ignore_checks)
{
    // target_pos = blimp.pos_ned;
    // target_yaw = blimp.ahrs.get_yaw();

    return true;
}

//Runs the main loiter controller
void ModeAuto::run()
{
    // const float dt = blimp.scheduler.get_last_loop_time_s();
}