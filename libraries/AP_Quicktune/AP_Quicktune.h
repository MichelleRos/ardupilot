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
    AP_Quicktune();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Quicktune);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AP_Quicktune *get_singleton() { return _singleton; }

    void update();

// private:

    static AP_Quicktune *_singleton;

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

    bool have_pilot_input();

    enum class axis_names : uint8_t {
        RLL = 0,
        PIT = 1,
        YAW = 2,
        DONE = 3,
    };

    // enum class param_suffixes : uint8_t {
    //     FF = 0,
    //     P = 1,
    //     I = 2,
    //     D = 3,
    //     SMAX = 4,
    //     FLTT = 5,
    //     FLTD = 6,
    //     FLTE = 7,
    // };

    enum class param_s : uint8_t {
        RLL_FF,
        RLL_P,
        RLL_I,
        RLL_D,
        RLL_SMAX,
        RLL_FLTT,
        RLL_FLTD,
        RLL_FLTE,
        PIT_FF,
        PIT_P,
        PIT_I,
        PIT_D,
        PIT_SMAX,
        PIT_FLTT,
        PIT_FLTD,
        PIT_FLTE,
        YAW_FF,
        YAW_P,
        YAW_I,
        YAW_D,
        YAW_SMAX,
        YAW_FLTT,
        YAW_FLTD,
        YAW_FLTE,
        END,
    };


    enum class stages : uint8_t {
        D = 0,
        P = 1,
    };

    uint8_t stage = 1;
    float last_stage_change = get_time();
    float last_gain_report = get_time();
    float last_pilot_input = get_time();
    float tune_done_time = 0; //nil
    float slew_parm = 0; //nil
    float slew_target = 0;
    float slew_delta = 0;

    uint32_t axes_done = 0;
    uint32_t filters_done = 0;

    uint32_t param_saved = 0; //{}
    uint32_t param_changed = 0; //{}
    bool need_restore = false;

    uint32_t last_warning = get_time();

    void reset_axes_done();
    void setup_SMAX();
    void setup_filters(axis_names axis);
    bool have_pilot_input();
    axis_names get_current_axis();
    float get_slew_rate(axis_names axis);
    int8_t advance_stage(axis_names axis);
    void adjust_gain(param_s param, float value, bool limit);
    float get_gain_mul();
    void restore_all_params();
    void save_all_params();
    bool reached_limit();
    void get_all_params();
    bool item_in_bitmask(uint8_t item, uint32_t bitmask);
    bool axis_done(axis_names axis);
    bool axis_enabled(uint8_t axis);
    bool filter_done(AP_Quicktune::axis_names axis);
    param_s get_pname(AP_Quicktune::axis_names axis, AP_Quicktune::stages stage);

    AP_Arming *arming = AP::arming().get_singleton();
    AP_Vehicle *vehicle = AP::vehicle();
    AP_Logger *logger = AP::logger().get_singleton();
};

namespace AP {
    AP_Quicktune *quicktune();
};

#endif  // QUICKTUNE_ENABLED
