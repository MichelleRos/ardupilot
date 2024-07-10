#pragma once

#ifndef QUICKTUNE_ENABLED
 #define QUICKTUNE_ENABLED 1
#endif

#if QUICKTUNE_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_AHRS/AP_AHRS.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>

#define UPDATE_RATE_HZ 40
#define STAGE_DELAY 4.0
#define PILOT_INPUT_DELAY 4.0
#define YAW_FLTE_MAX 2.0
#define FLTD_MUL 0.5
#define FLTT_MUL 0.5
#define DEFAULT_SMAX 50.0

#define OPTIONS_TWO_POSITION (1<<0)

class AP_Quicktune {
public:
    AP_Quicktune()
    {
        AP_Param::setup_object_defaults(this, var_info);
        if (singleton != nullptr) {
            AP_HAL::panic("Quicktune must be singleton.");
        }
        singleton = this;
    }

    static AP_Quicktune *get_singleton(void) {
        return singleton;
    }

    // Empty destructor to suppress compiler warning
    virtual ~AP_Quicktune() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Quicktune);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    void update();

// private:

    // parameters
    AP_Float enable;
    AP_Int8 axes_enabled;
    AP_Float double_time;
    AP_Float gain_margin;
    AP_Float osc_smax;
    AP_Float yaw_p_max;
    AP_Float yaw_d_max;
    AP_Float rp_pi_ratio;
    AP_Float y_pi_ratio;
    AP_Int8 auto_filter;
    AP_Float auto_save;
    AP_Float max_reduce;
    AP_Int16 options;

    float get_time() {
        return AP_HAL::millis() * 0.001;
    }

    enum class axis_names : uint8_t {
        RLL = 0,
        PIT = 1,
        YAW = 2,
        DONE = 3,
        END = 4,
    };

    enum class param_s : uint8_t {
        RLL_P,
        RLL_I,
        RLL_D,
        RLL_SMAX,
        RLL_FLTT,
        RLL_FLTD,
        RLL_FLTE,
        RLL_FF,
        PIT_P,
        PIT_I,
        PIT_D,
        PIT_SMAX,
        PIT_FLTT,
        PIT_FLTD,
        PIT_FLTE,
        PIT_FF,
        YAW_P,
        YAW_I,
        YAW_D,
        YAW_SMAX,
        YAW_FLTT,
        YAW_FLTD,
        YAW_FLTE,
        YAW_FF,
        END,
    };

    //Also the gains
    enum class stages : uint8_t {
        D,
        P,
        DONE,
        I,
        FF,
        SMAX,
        FLTT,
        FLTD,
        FLTE,
        END,
    };

    stages current_stage = stages::D;
    float last_stage_change = get_time();
    float last_gain_report = get_time();
    float last_pilot_input = get_time();
    float tune_done_time = 0; //nil
    param_s slew_parm = param_s::END; //nil
    float slew_target = 0;
    uint8_t slew_steps = 0;
    float slew_delta = 0;

    uint32_t axes_done = 0;
    uint32_t filters_done = 0;

    //Bitmask of changed parameters
    uint32_t param_changed = 0; //{}

    //Saved values of the parameter
    float param_saved[uint8_t(param_s::END)];

    bool need_restore = false;

    uint32_t last_warning = get_time();

    void reset_axes_done();
    void setup_SMAX();
    void setup_filters(axis_names axis);
    bool have_pilot_input();
    axis_names get_current_axis();
    float get_slew_rate(axis_names axis);
    void advance_stage(axis_names axis);
    void adjust_gain(param_s param, float value);
    void adjust_gain_limited(param_s param, float value);
    float get_gain_mul();
    void restore_all_params();
    void save_all_params();
    bool reached_limit();
    void get_all_params();
    bool item_in_bitmask(uint8_t item, uint32_t bitmask);
    param_s get_pname(axis_names axis, stages stage);
    float get_param_value(param_s param);
    void set_param_value(param_s param, float value);
    float gain_limit(param_s param);
    axis_names get_axis(param_s param);
    float limit_gain(param_s param, float value);
    const char* get_param_name(param_s param);
    void set_bitmask(bool value, uint32_t &bitmask, uint8_t position);
    stages get_stage(param_s param);
    const char* get_axis_name(axis_names axis);

    AP_Arming *arming = AP::arming().get_singleton();
    AP_Vehicle *vehicle = AP::vehicle();
    AP_Logger *logger = AP::logger().get_singleton();
    AP_InertialSensor *imu = AP_InertialSensor::get_singleton();
    const RCMapper* rcmap = AP::rcmap();
    AC_AttitudeControl *attitude_control = AC_AttitudeControl::get_singleton();

    static AP_Quicktune *singleton;
    
};

#endif  // QUICKTUNE_ENABLED
