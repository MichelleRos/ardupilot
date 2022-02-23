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

bool ModeSrcloc::init(bool ignore_checks){
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();
    right_mv = true;
    stage = 0;
    cast = true; //false means surge
    atwp = false;
    surging = false;
    push = 0.0f;
    fnd_pl = false;
    drift = false;

    return true;
}

//Runs the main srcloc controller
void ModeSrcloc::run()
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode srcloc");
    if(g.sl_mode == 1){ //Cast & surge
        if (blimp.plume_str_curr > (blimp.plume_strs[blimp.plume_arr_pos] * g.sl_plume_found)) {
            cast = false; //check for source, if found, set cast to false to go to surge
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Found plume.");
        }
        float distsq = blimp.pos_ned.distance_squared(target_pos);
        if (distsq < sq(g.wpnav_radius)) {
            atwp = true;
            surging = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached wp.");
        }

        if(cast && atwp){
            atwp = false;
            stage++;

            Vector3f add_tar;
            //Here, x is upwind, y is side to side
            add_tar.x = 0.2f;
            if (right_mv) {
                add_tar.y = g.sl_big * stage + g.sl_mul2;
                right_mv = false;
            }
            else {
                add_tar.y = -(g.sl_big * stage + g.sl_mul2);
                right_mv = true;
            }
            //assume north
            //g.sl_wind_deg
            target_pos = blimp.pos_ned + add_tar; //should it be currpos instead?
            if (right_mv) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Forward %0.2f, Left %0.2f, stage %d, tar %0.2f %0.2f plum %0.2f %0.2f", add_tar.x, -add_tar.y, stage, target_pos.x, target_pos.y, blimp.plume_str_curr, blimp.plume_strs[blimp.plume_arr_pos]*g.sl_plume_found);
            else          GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Casting: Forward %0.2f, Right %0.2f, stage %d, tar %0.2f %0.2f plum %0.2f %0.2f", add_tar.x, add_tar.y, stage, target_pos.x, target_pos.y, blimp.plume_str_curr, blimp.plume_strs[blimp.plume_arr_pos]*g.sl_plume_found);
        } else if (!cast && !surging) {
            cast = true;
            atwp = false;
            surging = true;
            stage = 0;

            Vector3f add_tar;
            add_tar.x = g.sl_surg_dist;
            //assume north
            target_pos = blimp.pos_ned + add_tar;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Surging. Forward %0.2f, tar %0.2f %0.2f", add_tar.x, target_pos.x, target_pos.y);
        }
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});

    //
    //Novel push & drift
    //
    } else if(g.sl_mode == 2){
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
    } else if(g.sl_mode == 3){
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
        blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
    }
}

float ModeSrcloc::levydis(float x){
    return sqrtf(c/(2*M_PI))*(expf(-c/(2*(x-mew)))/powf(x-mew,3.0f/2.0f));
}
