#include "Blimp.h"
/*
 * Init and run calls for land flight mode
 */

bool ModeLand::init(bool ignore_checks)
{
    control_position = blimp.position_ok();
    if (control_position) {
        target_pos = blimp.pos_ned;
        target_yaw = blimp.ahrs.get_yaw();
    }
    return true;
}

// Runs the main land controller
void ModeLand::run()
{
    if (control_position) {
        const float dt = blimp.scheduler.get_last_loop_time_s();
        if ((fabsf(target_pos.z-blimp.pos_ned.z) < (g.max_pos_z*blimp.loiter->pos_lag))) {
            target_pos.z += (0.5f * g.max_pos_z* dt);
        }
        blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});        
    } else {
        //No position, so all we can do is go down slowly.
        motors->right_out = 0;
        motors->front_out = 0;
        motors->yaw_out = 0;
        motors->down_out = 0.5;
    }
}

// set_mode_land_failsafe - sets mode to LAND
// this is always called from a failsafe so we trigger notification to pilot
void Blimp::set_mode_land_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}