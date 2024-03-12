#include "AP_Quicktune.h"

#if QUICKTUNE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>


const AP_Param::GroupInfo AP_Quicktune::var_info[] = {
    // @Param: QUIK_ENABLE
    // @DisplayName: Enable Quicktune
    // @Description:
    // @User: Standard
    AP_GROUPINFO("ENABLE", 0, AP_Quicktune, enabled, 0),

    // @Param: QUIK_AXES
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AP_Quicktune, axes, 7),

    // @Param: QUIK_ENABLE
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("DOUBLE_TIME", 2, AP_Quicktune, double_time, 10),

    // @Param: QUIK_GAIN_MARGIN
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("GAIN_MARGIN", 3, AP_Quicktune, gain_margin, 60),

    // @Param: QUIK_OSC_SMAX
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("OSC_SMAX", 4, AP_Quicktune, osc_smax, 5),

    // @Param: QUIK_YAW_P_MAX
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("YAW_P_MAX", 5, AP_Quicktune, yaw_p_max, 0.5),

    // @Param: QUIK_YAW_D_MAX
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("YAW_D_MAX", 6, AP_Quicktune, yaw_d_max, 0.01),

    // @Param: QUIK_RP_PI_RATIO
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("RP_PI_RATIO", 7, AP_Quicktune, rp_pi_ratio, 1.0),

    // @Param: QUIK_Y_PI_RATIO
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("Y_PI_RATIO", 8, AP_Quicktune, y_pi_ratio, 10),

    // @Param: QUIK_AUTO_FILTER
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("AUTO_FILTER", 9, AP_Quicktune, auto_filter, 1),

    // @Param: QUIK_AUTO_SAVE
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("AUTO_SAVE", 10, AP_Quicktune, auto_save, 0),

    // @Param: QUIK_RC_FUNC
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("RC_FUNC", 11, AP_Quicktune, rc_func, 300),

    // @Param: QUIK_MAX_REDUCE
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("MAX_REDUCE", 12, AP_Quicktune, max_reduce, 20),

    // @Param: QUIK_OPTIONS
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 13, AP_Quicktune, options, 0),

    AP_GROUPEND
};

// constructor
AP_Quicktune::AP_Quicktune()
{

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Quicktune must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}













// singleton instance
AP_Quicktune *AP_Quicktune::_singleton;

namespace AP {

AP_Quicktune *quicktune()
{
    return AP_Quicktune::get_singleton();
}

}
#endif //QUICKTUNE_ENABLED
