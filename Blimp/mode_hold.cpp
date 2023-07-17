#include "Blimp.h"
/*
 * Init and run calls for hold flight mode
 */

// Runs the main hold controller
void ModeHold::run()
{
    //Stop moving
    motors->right_out = 0;
    motors->front_out = 0;
    motors->yaw_out = 0;
    motors->down_out = 0;


    //TODO (for mode land):
    //_inav.get_position_z_up_cm() should work even without GPS (used within AC_PosControl.cpp for land_nogps in Copter.
    /*Or better to use this instead (to not use the inav filter, and to have things in metres):
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU
    }*/
}

// set_mode_hold_failsafe - sets mode to HOLD and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Blimp::set_mode_hold_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::HOLD, reason);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}