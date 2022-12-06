#include "Blimp.h"
/*
 * Init and run calls for srcloc flight mode
 */

//randf to return random number between 0 and 1
#define randf() (float)rand()/RAND_MAX

#define Lmin 1.5f //50 cm
#define mu 3.0f

bool ModeSrcloc::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();
    right_mv = true;
    stage = -1;
    cs = CS::CASTING_START;
    beta = 30;
    cast_time = 0.0f; //all times are in seconds
    lost_time = 0.0f;
    push = 0.0f;
    fnd_pl = false;
    drift = false;
    motors->right_out = 0.0f;
    motors->front_out = 0.0f;
    motors->down_out = 0.0f;
    motors->yaw_out = 0.0f;
    out = {0,0};

    return true;
}

//Runs the main srcloc controller
void ModeSrcloc::run()
{
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RSSI: %.2f", blimp.rssi.read_receiver_rssi());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode srcloc");

    if (blimp.plume_str_curr > g.sl_source_found) {
        set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found source. Finished.");
    }
    //
    // Cast & Surge - position controller version
    //
    if (g.sl_mode == (int)SLMode::CASTSURGEPOS) {
        switch (cs) {
        case CS::CASTING_RUN: {
            if (blimp.plume_str_curr > g.sl_plume_found) {
                cs = CS::SURGING_START;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume. Surging. %f %f", blimp.plume_str_curr, float(g.sl_plume_found));
            }
        } FALLTHROUGH;
        case CS::SURGING_RUN: {
            float distsq = blimp.pos_ned.distance_squared(target_pos);
            if (distsq < sq(g.wpnav_radius)) { //Should set surging distance far enough that it always loses the plume before getting to the waypoint.
                cs = CS::CASTING_START;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached wp. Casting.");
            } else if (blimp.plume_str_curr < g.sl_plume_found) {
                cs = CS::CASTING_START;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lost plume. Casting.");
            }
        } break;
        case CS::CASTING_START: {
            cs = CS::CASTING_RUN;
            stage++;
            Vector3f add_tar;
            //Here, x is upwind, y is side to side
            add_tar.x = 0.2f;
            if (right_mv) {
                add_tar.y = g.sl_mulp * (stage+1);
                right_mv = false;
            } else {
                add_tar.y = -(g.sl_mulp * (stage+1));
                right_mv = true;
            }
            add_tar.rotate_xy(g.sl_wind_deg * DEG_TO_RAD);
            target_pos = blimp.pos_ned + add_tar; //should it be currpos instead?
            if (right_mv) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Forward %0.2f, Left %0.2f, stage %d, tar %0.2f %0.2f plum %0.2f %0.2f", add_tar.x, -add_tar.y, stage, target_pos.x, target_pos.y, blimp.plume_str_curr, (float)g.sl_plume_found);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Forward %0.2f, Right %0.2f, stage %d, tar %0.2f %0.2f plum %0.2f %0.2f", add_tar.x, add_tar.y, stage, target_pos.x, target_pos.y, blimp.plume_str_curr, (float)g.sl_plume_found);
            }
        } break;
        case CS::SURGING_START: {
            cs = CS::SURGING_RUN;
            stage = -1;
            Vector3f add_tar;
            add_tar.x = g.sl_surg_dist;
            if (right_mv) {
                add_tar.rotate_xy((g.sl_wind_deg - beta) * DEG_TO_RAD);
            } else {
                add_tar.rotate_xy((g.sl_wind_deg + beta) * DEG_TO_RAD);
            }
            //assume north
            target_pos = blimp.pos_ned + add_tar;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Surging. Forward %0.2f, tar %0.2f %0.2f", add_tar.x, target_pos.x, target_pos.y);
        } break;
        }
        blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
        //
        //
        // Cast & Surge - accel/vel fin flap time version
        //
        //
    } else if (g.sl_mode == (int)SLMode::CASTSURGEACCEL || g.sl_mode == (int)SLMode::CASTSURGEVEL) {
        float now = AP_HAL::micros() * 1.0e-6;
        switch (cs) {
        case CS::CASTING_RUN: {
            if (blimp.plume_str_curr > g.sl_plume_found) {
                cs = CS::SURGING_START;
            }
            if ((!right_mv && blimp.vel_ned_filtd.y<g.sl_vel_start) || (right_mv && -blimp.vel_ned_filtd.y<g.sl_vel_start)) {
                //moving to the right and vel to the right is lower than param OR moving to the left and vel to the left is lower than param
                //wait for velocity in the opposite direction to casting direction to get close to zero before starting timer.
                //right_mv is opposite to what would be expected as it's set ready for the next move once CASTING_START has set fins
                cast_time = now;
                //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Resetting timer: rightmv %d, vel %f", right_mv, blimp.vel_ned_filtd.y);
            } // else GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting timer.");
            if ((now - cast_time) > (g.sl_push_time + stage*g.sl_mula)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached cast time - time %f, stage%d", g.sl_push_time + stage*g.sl_mula, stage);
                cs = CS::CASTING_START;
            }
            if ((now - cast_time) > (g.sl_push_time*0.8) && stage == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached cast time - time %f, stage%d", g.sl_push_time*0.8, stage);
                cs = CS::CASTING_START;
            }
        } break;
        case CS::SURGING_RUN: {
            if (blimp.plume_str_curr > g.sl_plume_found) {
                lost_time = now; //lost time is time since it lost the plume. Setting to current time whenever it finds the plume.
                // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Setting lost_time to curr.");
            }
            if ((now - lost_time) > (g.sl_drift_time)) {
                //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached surging time - %f", (float)g.sl_drift_time);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lost plume. Stopping surge: now %f losttime %f", now, lost_time);
                cs = CS::CASTING_START;
            }
        } break;
        case CS::CASTING_START: {
            cast_time = now;
            stage++;
            out.x = g.sl_thst_cf;
            if (right_mv) {
                out.y = g.sl_thst_cr;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Right.");
                right_mv = false;
            } else {
                out.y = -g.sl_thst_cr;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Left.");
                right_mv = true;
            }
            cs = CS::CASTING_RUN;
        } break;
        case CS::SURGING_START: {
            stage = -1;
            if ((now - cast_time) < 0.1f) { //essentially just needs to be less than the looprate since it's being reset by sl_vel_start logic
                right_mv = !right_mv;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Flipped right_mv to %d because of low cast time.", right_mv);
            }
            cast_time = now;
            lost_time = now;
            if (!right_mv) {
                out.y = g.sl_thst_sr;
            } else {
                out.y = -g.sl_thst_sr;
            }
            out.x = g.sl_thst_sf;
            //right_mv = !right_mv;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume. Surging. thrust F %0.2f, R %0.2f, str %f out of %f,", out.x, out.y, blimp.plume_str_curr, float(g.sl_plume_found));
            cs = CS::SURGING_RUN;
        } break;
        }
        if (g.sl_mode == (int)SLMode::CASTSURGEACCEL) {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Output a: %f %f", out.x, out.y);
            motors->front_out = out.x;
            motors->right_out = out.y;
        } else {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Output v: %f %f", out.x, out.y);
            Vector2f out2;
            out2 = out * g.max_vel_xy;
            blimp.rotate_BF_to_NE(out2);
            Vector2f actuator = blimp.pid_vel_xy.update_all(out2, blimp.vel_ned_filtd.xy(), {0,0});
            blimp.rotate_NE_to_BF(actuator);
            motors->front_out = actuator.x;
            motors->right_out = actuator.y;
        }
        target_yaw = g.sl_wind_deg * DEG_TO_RAD;
        blimp.loiter->run(target_pos, target_yaw, Vector4b{true,true,false,false});
        //Add wind capability for CS
        //Add "found "" radiuza

        //
        //Novel push & drift
        //
    } else if (g.sl_mode == (int)SLMode::PUSHDRIFT) {
        float now = AP_HAL::micros() * 1.0e-6;
        if (blimp.plume_str_curr > g.sl_plume_found) {
            fnd_pl = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume.");
        }
        if ((now - push) > (g.sl_push_time + g.sl_drift_time)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting push. Now=%0.2f Push=%0.2f", now, push);
            motors->front_out = 1.0f;
            push = now;
            drift = false;
        } else if ((now - push) > g.sl_push_time && drift == false) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting drift. Now=%0.2f Push=%0.2f", now, push);
            motors->front_out = 0.0f;
            drift = true;
        }
        target_yaw = g.sl_wind_deg * DEG_TO_RAD;
        blimp.loiter->run(target_pos, target_yaw, Vector4b{true,true,true,false});

        //
        // Levy Walk from rahbar_3-d_2017
        //
    } else if (g.sl_mode == (int)SLMode::LEVYWALK) {
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rand %0.8f, Levy %0.8f", randf()*10, levydis(randf()*10));
        float distsq = blimp.pos_ned.distance_squared(target_pos);
        if (distsq < sq(g.wpnav_radius)) {
            Vector3f next;
            float Ml = Lmin*powf(randf(),1/(1-mu));
            float Ta = randf()*2*M_PI;
            next.x = Ml*sinf(Ta);
            next.y = Ml*cosf(Ta);
            //blimp.rotate_BF_to_NE(next.xy());
            float dist_tar = sqrtf(next.distance_squared({0,0,0}));
            target_pos.x += next.x;
            target_pos.y += next.y;

            target_yaw = wrap_PI(Ta);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "New target %0.4f, %0.2f, %0.2f, %0.2f", dist_tar, target_pos.x, target_pos.y, target_yaw*RAD_TO_DEG);
        }
        if (blimp.plume_str_curr > g.sl_plume_found) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume. Switching to srcloc.");
            //This just sets it temporarily. A reboot resets it.
            g.sl_mode = (int)SLMode::CASTSURGEVEL;
        }
        blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
    }
}
