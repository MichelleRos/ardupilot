#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

 bool ModeGuided::init(bool ignore_checks){
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();

    return true;
 }

//Runs the main loiter controller
void ModeGuided::run()
{
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}

void ModeGuided::set_target(Location tar) //set target from lat long alt location, TODO: Add option to do yaw as well.
{
    Vector3f tar_vec;
    if(tar.get_vector_from_origin_NEU(tar_vec)){ //Unfortunately returns vector in cm instead of m...
        target_pos.x = tar_vec.x/100.0;
        target_pos.y = tar_vec.y/100.0;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Received target: %f, %f", target_pos.x, target_pos.y);
    }
    //Don't set target if origin not set
}
