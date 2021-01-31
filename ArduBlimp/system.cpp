#include "Blimp.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    blimp.failsafe_check();
}

void Blimp::init_ardupilot()
{

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    BoardConfig.init();


    // initialise notify system
    notify.init();
    notify_flight_mode();

    // initialise battery monitor
    battery.init();

    // Init RSSI
    rssi.init();
    
    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts();

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // update motor interlock state
    // update_using_interlock();


    init_rc_in();               // sets up rc channels from radio

    // allocate the motors class
    allocate_motors();

    // initialise rc channels including setting mode
    rc().init();

    // sets up motors and output to escs
    init_rc_out();


    // motors initialised so parameters can be sent
    ap.initialised_params = true;

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // attitude_control->parameter_sanity_check();
    // pos_control->set_dt(scheduler.get_loop_period_s());

#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs().send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
        delay(1000);
    }

    // set INS to HIL mode
    ins.set_hil_mode();
#endif

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

// #if MODE_AUTO_ENABLED == ENABLED
//     // initialise mission library
//     mode_auto.mission.init();
// #endif

    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&blimp, &Blimp::Log_Write_Vehicle_Startup_Messages, void));

    startup_INS_ground();

#ifdef ENABLE_SCRIPTING
    g2.scripting.init();
#endif // ENABLE_SCRIPTING

    // set landed flags
    // set_land_complete(true);
    // set_land_complete_maybe(true);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    // failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // setup fin output
    motors->setup_fins();

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // attempt to switch to MANUAL, if this fails then switch to Land
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        // set mode to STABILIZE will trigger mode change notification to pilot
        set_mode(Mode::Number::MANUAL, ModeReason::UNAVAILABLE);
    } else {
        // alert pilot to mode change
        AP_Notify::events.failsafe_mode_change = 1;
    }

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Blimp::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Blimp::position_ok() const
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// ekf_has_absolute_position - returns true if the EKF can provide an absolute WGS-84 position estimate
bool Blimp::ekf_has_absolute_position() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors->armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// ekf_has_relative_position - returns true if the EKF can provide a position estimate relative to it's starting position
bool Blimp::ekf_has_relative_position() const
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
// #if OPTFLOW == ENABLED
//     if (optflow.enabled()) {
//         enabled = true;
//     }
// #endif
// #if HAL_VISUALODOM_ENABLED
//     if (visual_odom.enabled()) {
//         enabled = true;
//     }
// #endif
    if (!enabled) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    if (!motors->armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
}

// returns true if the ekf has a good altitude estimate (required for modes which do AltHold)
bool Blimp::ekf_alt_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow alt control with only dcm
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // require both vertical velocity and position
    return (filt_status.flags.vert_vel && filt_status.flags.vert_pos);
}

// update_auto_armed - update status of auto_armed flag
void Blimp::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
        // if heliblimps are on the ground, and the motor is switched off, auto-armed should be false
        // so that rotor runup is checked again before attempting to take-off
        if(ap.land_complete && motors->get_spool_state() != Fins::SpoolState::THROTTLE_UNLIMITED && ap.using_interlock) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        
        // // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        // if(motors->armed() && ap.using_interlock) {
        //     if(!ap.throttle_zero && motors->get_spool_state() == Fins::SpoolState::THROTTLE_UNLIMITED) {
        //         set_auto_armed(true);
        //     }
        // // if motors are armed and throttle is above zero auto_armed should be true
        // // if motors are armed and we are in throw mode, then auto_armed should be true
        // } else if (motors->armed() && !ap.using_interlock) {
        //     if(!ap.throttle_zero) {
        //         set_auto_armed(true);
        //     }
        // }
    }
}

/*
  should we log a message type now?
 */
bool Blimp::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
#else
    return false;
#endif
}

// return MAV_TYPE corresponding to frame class
MAV_TYPE Blimp::get_frame_mav_type()
{
    return MAV_TYPE_QUADROTOR; //MIR changed to this for now - will need to deal with mavlink changes later
}

// return string corresponding to frame_class
const char* Blimp::get_frame_string()
{
    return "AIRFISH";
}

/*
  allocate the motors class
 */
void Blimp::allocate_motors(void)
{
    switch ((Fins::motor_frame_class)g2.frame_class.get()) {
        case Fins::MOTOR_FRAME_AIRFISH:
        default:
            motors = new Fins(blimp.scheduler.get_loop_rate_hz());
            // motors_var_info = Fins::var_info; //MIR need to deal with this.
            break;
    }
    if (motors == nullptr) {
        AP_HAL::panic("Unable to allocate FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // const struct AP_Param::GroupInfo *ac_var_info;

    // attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
    // ac_var_info = AC_AttitudeControl_Multi::var_info;
    // if (attitude_control == nullptr) {
    //     AP_HAL::panic("Unable to allocate AttitudeControl");
    // }
    // AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);
        
    // pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);
    // if (pos_control == nullptr) {
    //     AP_HAL::panic("Unable to allocate PosControl");
    // }
    // AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

    // wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);

    // if (wp_nav == nullptr) {
    //     AP_HAL::panic("Unable to allocate WPNav");
    // }
    // AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    // loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    // if (loiter_nav == nullptr) {
    //     AP_HAL::panic("Unable to allocate LoiterNav");
    // }
    // AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);

    // param count could have changed
    AP_Param::invalidate_count();
}
