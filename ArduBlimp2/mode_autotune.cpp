#include "Blimp.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED == ENABLED

bool AutoTune::init()
{
    // use position hold while tuning if we were in QLOITER
    bool position_hold = (blimp.control_mode == Mode::Number::LOITER || blimp.control_mode == Mode::Number::POSHOLD);

    return init_internals(position_hold,
                          blimp.attitude_control,
                          blimp.pos_control,
                          blimp.ahrs_view,
                          &blimp.inertial_nav);
}

/*
  start autotune mode
 */
bool AutoTune::start()
{
    // only allow flip from Stabilize, AltHold,  PosHold or Loiter modes
    if (blimp.control_mode != Mode::Number::STABILIZE &&
        blimp.control_mode != Mode::Number::ALT_HOLD &&
        blimp.control_mode != Mode::Number::LOITER &&
        blimp.control_mode != Mode::Number::POSHOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (blimp.ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!blimp.motors->armed() || !blimp.ap.auto_armed || blimp.ap.land_complete) {
        return false;
    }

    return AC_AutoTune::start();
}

void AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    blimp.update_simple_mode();

    // reset target lean angles and heading while landed
    if (blimp.ap.land_complete) {
        // we are landed, shut down
        float target_climb_rate = get_pilot_desired_climb_rate_cms();

        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            blimp.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            blimp.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }
        blimp.attitude_control->reset_rate_controller_I_terms_smoothly();
        blimp.attitude_control->set_yaw_target_to_current_heading();

        float target_roll, target_pitch, target_yaw_rate;
        get_pilot_desired_rp_yrate_cd(target_roll, target_pitch, target_yaw_rate);

        blimp.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        blimp.pos_control->relax_alt_hold_controllers(0.0f);
        blimp.pos_control->update_z_controller();
    } else {
        // run autotune mode
        AC_AutoTune::run();
    }
}


/*
  get stick input climb rate
 */
float AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate = blimp.get_pilot_desired_climb_rate(blimp.channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = blimp.mode_autotune.get_avoidance_adjusted_climbrate(target_climb_rate);

    return target_climb_rate;
}

/*
  get stick roll, pitch and yaw rate
 */
void AutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    blimp.mode_autotune.get_pilot_desired_lean_angles(des_roll_cd, des_pitch_cd, blimp.aparm.angle_max,
                                                       blimp.attitude_control->get_althold_lean_angle_max());
    yaw_rate_cds = blimp.mode_autotune.get_pilot_desired_yaw_rate(blimp.channel_yaw->get_control_in());
}

/*
  setup z controller velocity and accel limits
 */
void AutoTune::init_z_limits()
{
    blimp.pos_control->set_max_speed_z(-blimp.get_pilot_speed_dn(), blimp.g.pilot_speed_up);
    blimp.pos_control->set_max_accel_z(blimp.g.pilot_accel_z);
}

void AutoTune::log_pids()
{
    blimp.logger.Write_PID(LOG_PIDR_MSG, blimp.attitude_control->get_rate_roll_pid().get_pid_info());
    blimp.logger.Write_PID(LOG_PIDP_MSG, blimp.attitude_control->get_rate_pitch_pid().get_pid_info());
    blimp.logger.Write_PID(LOG_PIDY_MSG, blimp.attitude_control->get_rate_yaw_pid().get_pid_info());
}

/*
  check if we have a good position estimate
 */
bool AutoTune::position_ok()
{
    return blimp.position_ok();
}

/*
  initialise autotune mode
*/
bool ModeAutoTune::init(bool ignore_checks)
{
    return blimp.autotune.init();
}


void ModeAutoTune::run()
{
    blimp.autotune.run();
}

void ModeAutoTune::save_tuning_gains()
{
    blimp.autotune.save_tuning_gains();
}

void ModeAutoTune::stop()
{
    blimp.autotune.stop();
}

void ModeAutoTune::reset()
{
    blimp.autotune.reset();
}

#endif  // AUTOTUNE_ENABLED == ENABLED
