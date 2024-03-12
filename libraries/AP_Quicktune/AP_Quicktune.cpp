#include "AP_Quicktune.h"

#if QUICKTUNE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>


const AP_Param::GroupInfo AP_Quicktune::var_info[] = {
    // @Param: TOTAL
    // @DisplayName:  Total
    // @Description: total
    // @User: Standard
    AP_GROUPINFO("TOTAL", 0, AP_Quicktune, _var, 0),


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
