#pragma once

#ifndef QUICKTUNE_ENABLED
 #define QUICKTUNE_ENABLED 1
#endif

#if QUICKTUNE_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>

class AP_Quicktune {
public:
    AP_Quicktune();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Quicktune);







    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // get singleton instance
    static AP_Quicktune *get_singleton() { return _singleton; }

private:
    static AP_Quicktune *_singleton;

    // parameters
    AP_Float enabled;
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
    AP_Float rc_func;
    AP_Float max_reduce;
    AP_Float options;

};

namespace AP {
    AP_Quicktune *quicktune();
};

#endif  // QUICKTUNE_ENABLED
