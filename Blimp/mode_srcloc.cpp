#include "Blimp.h"
/*
 * Init and run calls for srcloc flight mode
 */

//randf to return random number between 0 and 1
#define randf() (float)rand()/RAND_MAX

#define c 1.0f
#define mew 0.0f

#define Lmin 1.5f //50 cm
#define mu 3.0f

#if defined(__GNUC__) &&  __GNUC__ >= 7 || defined(__clang_major__) && __clang_major__ >= 10
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#endif

bool ModeSrcloc::init(bool ignore_checks){
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();
    right_mv = true;
    stage = -1;
    cs = CS::CASTING_START;
    beta = 30;
    cast_time = 0.0f;
    push = 0.0f;
    fnd_pl = false;
    drift = false;
    motors->right_out = 0.0f;
    motors->front_out = 0.0f;
    motors->down_out = 0.0f;
    motors->yaw_out = 0.0f;

    return true;
}

//Runs the main srcloc controller
void ModeSrcloc::run()
{
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RSSI: %.2f", blimp.rssi.read_receiver_rssi());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode srcloc");

    if (blimp.plume_str_curr > g.sl_source_found){
        set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found source. Finished.");
    }
    //
    // Cast & Surge - position controller version
    //
    if(g.sl_mode == (int)SLMode::CASTSURGEPOS){
        switch(cs){
            case CS::CASTING_RUN:{
                //if (blimp.plume_str_curr > (blimp.plume_strs[blimp.plume_arr_pos] * g.sl_plume_found)){
                if (blimp.plume_str_curr > g.sl_plume_found){
                    cs = CS::SURGING_START;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume. Surging. %f %f", blimp.plume_str_curr, float(g.sl_plume_found));
                }
            } //No break so casting run also does wp check.
            case CS::SURGING_RUN:{
                float distsq = blimp.pos_ned.distance_squared(target_pos);
                if (distsq < sq(g.wpnav_radius)) { //Should set surging distance far enough that it always loses the plume before getting to the waypoint.
                    cs = CS::CASTING_START;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached wp. Casting.");
                }
                else if (blimp.plume_str_curr < g.sl_plume_found) {
                    cs = CS::CASTING_START;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lost plume. Casting.");
                }
            }break;
            case CS::CASTING_START: {
                cs = CS::CASTING_RUN;
                stage++;
                Vector3f add_tar;
                //Here, x is upwind, y is side to side
                add_tar.x = 0.2f;
                if (right_mv) {
                    add_tar.y = g.sl_mulp * (stage+1);
                    right_mv = false;
                }
                else {
                    add_tar.y = -(g.sl_mulp * (stage+1));
                    right_mv = true;
                }
                add_tar.rotate_xy(g.sl_wind_deg * DEG_TO_RAD);
                target_pos = blimp.pos_ned + add_tar; //should it be currpos instead?
                if (right_mv) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Forward %0.2f, Left %0.2f, stage %d, tar %0.2f %0.2f plum %0.2f %0.2f", add_tar.x, -add_tar.y, stage, target_pos.x, target_pos.y, blimp.plume_str_curr, blimp.plume_strs[blimp.plume_arr_pos]*g.sl_plume_found);
                else          GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Forward %0.2f, Right %0.2f, stage %d, tar %0.2f %0.2f plum %0.2f %0.2f", add_tar.x, add_tar.y, stage, target_pos.x, target_pos.y, blimp.plume_str_curr, blimp.plume_strs[blimp.plume_arr_pos]*g.sl_plume_found);
            }break;
            case CS::SURGING_START:{
                cs = CS::SURGING_RUN;
                stage = -1;
                Vector3f add_tar;
                add_tar.x = g.sl_surg_dist;
                if(right_mv)
                    add_tar.rotate_xy((g.sl_wind_deg - beta) * DEG_TO_RAD);
                else
                    add_tar.rotate_xy((g.sl_wind_deg + beta) * DEG_TO_RAD);
                //assume north
                target_pos = blimp.pos_ned + add_tar;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Surging. Forward %0.2f, tar %0.2f %0.2f", add_tar.x, target_pos.x, target_pos.y);
            }break;
        }
        blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
    //
    //
    // Cast & Surge - accel/fin flap time version
    //
    //
    } else if(g.sl_mode == (int)SLMode::CASTSURGEACCEL){
        float now = AP_HAL::micros() * 1.0e-6;
        switch(cs){
            case CS::CASTING_RUN:{
                //if (blimp.plume_str_curr > (blimp.plume_strs[blimp.plume_arr_pos] * g.sl_plume_found)){
                if (blimp.plume_str_curr > g.sl_plume_found){
                    cs = CS::SURGING_START;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume. Surging. %f %f", blimp.plume_str_curr, float(g.sl_plume_found));
                }
                if ((!right_mv && blimp.vel_ned_filtd.y<g.sl_vel_stop) || (right_mv && blimp.vel_ned_filtd.y>g.sl_vel_stop)){ //wait for left-right velocity to neutralise before starting timer.
                //right_mv is opposite to what would be expected as it's set ready for the next move once CASTING_START has set fins
                    cast_time = now;
                    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Resetting timer. %d", right_mv);
                } // else GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting timer.");
                if ((now - cast_time) > (g.sl_push_time + stage*g.sl_mula)) {
                    cs = CS::CASTING_START;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached cast time - %f, %d", g.sl_push_time + stage*g.sl_mula, stage);
                }
                if ((now - cast_time) > (g.sl_push_time*0.8) && stage == 0) {
                    cs = CS::CASTING_START;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached cast time - %f, %d", g.sl_push_time*0.8, stage);
                }
            } break;
            case CS::SURGING_RUN:{
                if(blimp.plume_str_curr < g.sl_plume_found){
                //if ((now - cast_time) > (g.sl_drift_time)) {
                    cs = CS::CASTING_START;
                    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached surging time - %f", (float)g.sl_drift_time);
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Lost plume. Stopping surge - %f", (float)g.sl_drift_time);
                }
            }break;
            case CS::CASTING_START: {
                cs = CS::CASTING_RUN;
                cast_time = now;
                stage++;
                motors->front_out = g.sl_thst_cf;
                if (right_mv) {
                    motors->right_out = g.sl_thst_cr;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Right.");
                    right_mv = false;
                }
                else {
                    motors->right_out = -g.sl_thst_cr;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Left.");
                    right_mv = true;
                }
            }break;
            case CS::SURGING_START:{
                cs = CS::SURGING_RUN;
                stage = -1;
                cast_time = now;
                if(right_mv)
                    motors->right_out = g.sl_thst_sr;
                else
                    motors->right_out = -g.sl_thst_sr;
                motors->front_out = g.sl_thst_sf;
                right_mv = !right_mv;
            }break;
        }
        blimp.loiter->run({0,0,target_pos.z}, g.sl_wind_deg * DEG_TO_RAD, Vector4b{true,true,false,false});
    //Add wind capability for CS
    //Add "found "" radiuza

    //
    //Novel push & drift
    //
    } else if(g.sl_mode == (int)SLMode::PUSHDRIFT){
        float now = AP_HAL::micros() * 1.0e-6;
        if (blimp.plume_str_curr > (blimp.plume_strs[blimp.plume_arr_pos] * g.sl_plume_found)) {
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
        blimp.loiter->run({0,0,0}, g.sl_wind_deg * DEG_TO_RAD, Vector4b{true,true,true,false});

    //
    // Levy Walk from rahbar_3-d_2017
    //
    } else if(g.sl_mode == (int)SLMode::LEVYWALK){
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rand %0.8f, Levy %0.8f", randf()*10, levydis(randf()*10));
        float distsq = blimp.pos_ned.distance_squared(target_pos);
        if (distsq < sq(g.wpnav_radius)){
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
        if (blimp.plume_str_curr > g.sl_plume_found){
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume. Switching to srcloc.");
            //This just sets it temporarily. A reboot resets it.
            g.sl_mode = 2;
        }
        blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
    }
}

float ModeSrcloc::levydis(float x){
    return sqrtf(c/(2*M_PI))*(expf(-c/(2*(x-mew)))/powf(x-mew,3.0f/2.0f));
}
