#include "Blimp.h"
/*
 * Init and run calls for level flight mode
 */

// Runs the main level controller
void ModeLevel::run()
{
    const float dt = blimp.scheduler.get_last_loop_time_s();
    Vector3f pilot;
    float pilot_yaw, level_pitch, level_roll;
    get_pilot_input(pilot, pilot_yaw);

    // Run leveling pids
    level_pitch = blimp.loiter->pid_lvl_pitch.update_all(0, ahrs.get_pitch(), dt, 0);
    level_roll = blimp.loiter->pid_lvl_roll.update_all(0, ahrs.get_roll(), dt, 0);

    float out_right_lvl = level_roll * blimp.loiter->level_max;
    float out_front_lvl = level_pitch * blimp.loiter->level_max;
    float out_right_com = pilot.y*g.max_man_thr;
    float out_front_com = pilot.x*g.max_man_thr;

    float totalr = out_right_lvl + out_right_com;
    if (totalr > motors->thr_max) {
        out_right_com = out_right_com - (totalr - motors->thr_max);
    }

    float totalf = out_front_lvl + out_front_com;
    if (totalf > motors->thr_max) {
        out_front_com = out_front_com - (totalf - motors->thr_max);
    }

    if (!blimp.motors->armed()) {
        blimp.loiter->pid_lvl_roll.set_integrator(0);
        blimp.loiter->pid_lvl_pitch.set_integrator(0);
    }

    motors->right_out = out_right_com + out_right_lvl;
    motors->front_out = out_front_com + out_front_lvl;
    motors->yaw_out = pilot_yaw*g.max_man_thr;
    motors->down_out = pilot.z*g.max_man_thr;

    AP::logger().WriteStreaming("LVL", "TimeUS,lr,lf,orl,ofl,orc,ofc", "Qffffff",
                                               AP_HAL::micros64(),
                                               level_roll,
                                               level_pitch,
                                               out_right_lvl,
                                               out_front_lvl,
                                               out_right_com,
                                               out_front_com);
}
