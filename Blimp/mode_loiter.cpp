#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

bool ModeLoiter::init(bool ignore_checks)
{
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
    pilot.x *= g.max_pos_x * dt;
    pilot.y *= g.max_pos_y * dt;
    pilot.z *= g.max_pos_z * dt;
    pilot_yaw *= g.max_pos_yaw * dt;

    if (g.simple_mode == 0) {
        //If simple mode is disabled, input is in body-frame, thus needs to be rotated.
        blimp.rotate_BF_to_NE(pilot.xy());
    }

    // This keeps the target position from getting too far away from the blimp's actual position.
    if ((fabsf(target_pos.x-blimp.pos_ned.x) < (g.max_pos_x*blimp.loiter->pos_lag)) || ((target_pos.x > blimp.pos_ned.x) && (pilot.x < 0)) || ((target_pos.x < blimp.pos_ned.x) && (pilot.x > 0))) {
        target_pos.x += pilot.x;
    }
    if ((fabsf(target_pos.y-blimp.pos_ned.y) < (g.max_pos_y*blimp.loiter->pos_lag)) || ((target_pos.y > blimp.pos_ned.y) && (pilot.y < 0)) || ((target_pos.y < blimp.pos_ned.y) && (pilot.y > 0))) {
        target_pos.y += pilot.y;
    }
    if ((fabsf(target_pos.z-blimp.pos_ned.z) < (g.max_pos_z*blimp.loiter->pos_lag)) || ((target_pos.z > blimp.pos_ned.z) && (pilot.z < 0)) || ((target_pos.z < blimp.pos_ned.z) && (pilot.z > 0))) {
        target_pos.z += pilot.z;
    }
    //Need to not just check target is greater than current, but also that it's not .too much. greater (eg not more than double pos lag).
    // if ((fabsf(wrap_PI(target_yaw-ahrs.get_yaw())) < (g.max_pos_yaw*blimp.loiter->pos_lag)) || ((target_yaw > ahrs.get_yaw()) && (target_yaw < wrap_PI(ahrs.get_yaw()+0.5)) && (pilot_yaw < 0)) || ((target_yaw < ahrs.get_yaw()) && (target_yaw > wrap_PI(ahrs.get_yaw()+0.5)) && (pilot_yaw > 0))) {
    //     target_yaw = wrap_PI(target_yaw + pilot_yaw);
    // }
    if (fabsf(wrap_PI(target_yaw-ahrs.get_yaw())) < (g.max_pos_yaw*blimp.loiter->pos_lag)) {
        target_yaw = wrap_PI(target_yaw + pilot_yaw);
    }

    if (!blimp.motors->armed()) {
        target_pos = blimp.pos_ned;
        target_yaw = blimp.ahrs.get_yaw();
    }

    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
