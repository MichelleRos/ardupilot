#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

//Number of seconds of movement that the target position can be ahead of actual position.
#define POS_LAG 1

 bool ModeLoiter::init(bool ignore_checks){
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();

    return true;
 }

//Runs the main loiter controller
void ModeLoiter::run()
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);
    pilot.x *= g.max_pos_xy * blimp.scheduler.get_loop_period_s();
    pilot.y *= g.max_pos_xy * blimp.scheduler.get_loop_period_s();
    pilot.z *= g.max_pos_z * blimp.scheduler.get_loop_period_s();
    pilot_yaw *= g.max_pos_yaw * blimp.scheduler.get_loop_period_s();

    if (g.simple_mode == 0){
        //If simple mode is disabled, input is in body-frame, thus needs to be rotated.
        blimp.rotate_BF_to_NE(pilot.xy());
    }
    //MIR Warning: this means that if PID_DZ param is greater than eg MAX_POS_XY param * POS_LAG then the blimp won't move at all.
    if(fabsf(target_pos.x-blimp.pos_ned.x) < (g.max_pos_xy*POS_LAG)) target_pos.x += pilot.x;
    if(fabsf(target_pos.y-blimp.pos_ned.y) < (g.max_pos_xy*POS_LAG)) target_pos.y += pilot.y;
    if(fabsf(target_pos.z-blimp.pos_ned.z) < (g.max_pos_z*POS_LAG)) target_pos.z += pilot.z;
    if(fabsf(wrap_PI(target_yaw-ahrs.get_yaw())) < (g.max_pos_yaw*POS_LAG)) target_yaw = wrap_PI(target_yaw + pilot_yaw);

    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
