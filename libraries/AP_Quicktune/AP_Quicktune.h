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

    virtual bool is_valid(const Location &rally_point) const { return true; }

    // parameters
    AP_Float _var;
};

namespace AP {
    AP_Quicktune *quicktune();
};

#endif  // QUICKTUNE_ENABLED
