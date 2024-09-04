#include "Blimp.h"
/*
 * Init and run calls for rtl flight mode
 */

bool ModeRTL::init(bool ignore_checks)
{
    Vector3f home_ned;
    IGNORE_RETURN(ahrs.get_relative_position_NED_home(home_ned));
    
    target_pos = blimp.pos_ned-home_ned;
    target_yaw = 0;
    return true;
}

//Runs the main rtl controller
void ModeRTL::run()
{
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
