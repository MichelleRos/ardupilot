#include "Blimp.h"

/*
 * Init and run calls for stabilize flight mode
 */

// manual_run - runs the main manual controller
// should be called at 100hz or more
void ModeManual::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_accelerations(target_roll, target_pitch, blimp.aparm.angle_max, blimp.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(Fins::DesiredSpoolState::SHUT_DOWN);
    } else {
        motors->set_desired_spool_state(Fins::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    roll_out = target_roll;
    pitch_out = target_pitch;
    yaw_out = target_yaw_rate;
    throttle_out = get_pilot_desired_throttle();
    motors->output();

    // // call attitude controller
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // // output pilot's throttle
    // attitude_control->set_throttle_out(get_pilot_desired_throttle(),
    //                                    true,
    //                                    g.throttle_filt);
}
