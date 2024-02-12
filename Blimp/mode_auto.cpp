#include "Blimp.h"
/*
 * Init and run calls for auto flight mode
 */

bool ModeAuto::init(bool ignore_checks)
{
    // target_pos = blimp.pos_ned;
    // target_yaw = blimp.ahrs.get_yaw();

    return true;
}

//Runs the main loiter controller
void ModeAuto::run()
{
    // const float dt = blimp.scheduler.get_last_loop_time_s();



    Vector3f target_pos = blimp.pos_ned;
    float target_yaw = blimp.ahrs.get_yaw();
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});




}

bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{



    return true;
}

bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{


    return true;
}

void ModeAuto::exit_mission()
{


}
