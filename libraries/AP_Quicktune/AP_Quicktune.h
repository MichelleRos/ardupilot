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
    AP_Float axes;
    AP_Float double_time;
    AP_Float gain_margin;
    AP_Float osc_smax;
    AP_Float yaw_p_max;
    AP_Float yaw_d_max;
    AP_Float rp_pi_ratio;
    AP_Float y_pi_ratio;
    AP_Float auto_filter;
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
    };

    enum class param_suffixes : uint8_t {
        FF = 0,
        P = 1,
        I = 2,
        D = 3,
        SMAX = 4,
        FLTT = 5,
        FLTD = 6,
        FLTE = 7,
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

    uint8_t axes_done = 0;
    uint8_t filters_done = 0;

    uint8_t params = 0; // {}
    uint8_t param_saved = 0; //{}
    uint8_t param_changed = 0; //{}
    bool need_restore = false;

    uint32_t last_warning = get_time();

    void reset_axes_done();
    void setup_SMAX();
    void setup_filters(axis_names axis);
    bool have_pilot_input();
    bool axis_enabled(axis_names axis);
    axis_names get_current_axis();
    float get_slew_rate(axis_names axis);
    int8_t advance_stage(axis_names axis);
    void adjust_gain(axis_names axis, param_suffixes suffix, float value, bool limit);
    float get_gain_mul();
    void restore_all_params();
    void save_all_params();
    bool reached_limit();

    AP_Arming *arming = AP::arming().get_singleton();
    AP_Vehicle *vehicle = AP::vehicle();
    AP_Logger *logger = AP::logger().get_singleton();
};

namespace AP {
    AP_Quicktune *quicktune();
};

#endif  // QUICKTUNE_ENABLED
