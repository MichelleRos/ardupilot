#include "Blimp.h"
/*
 * Init and run calls for hold flight mode
 */

// Runs the main hold controller
void ModeLand::run()
{
    //Stop moving
    motors->right_out = 0;
    motors->front_out = 0;
    motors->yaw_out = 0;
    motors->down_out = 0;
}

// set_mode_hold_failsafe - sets mode to HOLD and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Blimp::set_mode_hold_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::HOLD, reason);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}