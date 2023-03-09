#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

bool ModePSO::init(bool ignore_checks)
{
    target_vel = {0,0,0};
    target_yaw = 0;
    gbest = 1;
    for (uint8_t i=0; i<PAR_MAX; i++) X[i].zero();
    for (uint8_t i=0; i<PAR_MAX; i++) pbest[i].zero();
    return true;
}

#define self int(g.sysid_this_mav)-1
//Using this "ran" macro so that random number is regenerated for each new use
#define randf (float)rand()/RAND_MAX
//X is always synced with most recently seen plume strength's position, strength, and time for both messages.
//Note that all positions are relative to the current blimp's home.


void ModePSO::send_debug_loc(const char *name, Location value)
{
    mavlink_debug_loc_t packet {};
    packet.time_boot_ms = AP_HAL::millis();
    packet.lat = value.lat;
    packet.lon = value.lng;
    packet.alt = value.alt;
    memcpy(packet.name, name, MIN(strlen(name), (uint8_t)MAVLINK_MSG_DEBUG_LOC_FIELD_NAME_LEN));

    gcs().send_to_active_channels(MAVLINK_MSG_ID_DEBUG_LOC,
                                  (const char *)&packet);
}

float ModePSO::sgn(float x)
{
    if (x < 0) {
        return -1.0f;
    }
    else if (x > 0)
    {
        return 1.0f;
    }
    else
    {
        return 0.0f;
    }
}

#define ld 1.0f
//Runs the main loiter controller
void ModePSO::run()
{
    Vector2f A[PAR_MAX];
    Vector2f top;
    Vector2f bot;

    if (new_pos_recd && new_plu_recd){
        //Only calculate new target velocity when it actually has new information.
        new_pos_recd = false;
        new_plu_recd = false;

        for (uint8_t i=0; i<max_seen; i++){
            for (uint8_t j=0; j<max_seen; j++){
                if (i == j) continue;
                float dist = distance(i,j);
                if (dist < g.pso_min_dist){
                    top.x = ld*(X[i].x - X[j].x);
                    top.y = ld*(X[i].y - X[j].y);
                    bot.x = dist*fabsf(X[i].x - X[j].x);
                    bot.y = dist*fabsf(X[i].y - X[j].y);
                    AP::logger().WriteStreaming("PSOD", "TimeUS,i,j,tx,ty,bx,by,d", "s#------", "F-------",
                                                "QBBfffff", AP_HAL::micros64(),
                                                i,j,top.x,top.y,bot.x,bot.y,dist);
                    if(!is_zero(bot.x)) A[i].x += (top.x/bot.x);
                    if(!is_zero(bot.y)) A[i].y += (top.y/bot.y);
                }
            }
        }

        AP::logger().WriteStreaming("PSOW", "TimeUS,vx,vy,pbx,pby,gbx,gby,avx,avy", "Qffffffff",
                                    AP_HAL::micros64(),
                                    target_vel.x, target_vel.y, pbest[self].x - X[self].x, pbest[self].y - X[self].y, pbest[gbest].x - X[self].x, pbest[gbest].y - X[self].y, A[self].x, A[self].y);

        target_vel.x = g.pso_w_vel*target_vel.x + g.pso_w_per_best*randf*sgn(pbest[self].x - X[self].x) + g.pso_w_glo_best*randf*sgn(pbest[gbest].x - X[self].x) + g.pso_w_avoid*randf*sgn(A[self].x);
        target_vel.y = g.pso_w_vel*target_vel.y + g.pso_w_per_best*randf*sgn(pbest[self].y - X[self].y) + g.pso_w_glo_best*randf*sgn(pbest[gbest].y - X[self].y) + g.pso_w_avoid*randf*sgn(A[self].y);

        target_vel.x = constrain_float(target_vel.x,-g.pso_speed_limit,g.pso_speed_limit);
        target_vel.y = constrain_float(target_vel.y,-g.pso_speed_limit,g.pso_speed_limit);
    }

    blimp.loiter->run_vel(target_vel, target_yaw, Vector4b{false,false,false,false});
    AP::logger().WriteStreaming("PSOT", "TimeUS,tvx,tvy", "Qff",
                                    AP_HAL::micros64(),
                                    target_vel.x, target_vel.y);

    uint32_t now = AP_HAL::millis();
    if((now - bests_sent) > 1000){
        bests_sent = now;
        for (int i=0; i<max_seen; i++){
            //For some mad reason, this x and y is in cm
            Location pbestLatLng{Vector3f{pbest[i].x*100,pbest[i].y*100, 0.0f},Location::AltFrame::ABSOLUTE};
            char nm[10];
            hal.util->snprintf(nm, sizeof(nm), "PBEST%d", i+1);
            send_debug_loc(nm, pbestLatLng);

            // gcs().send_named_float(nm, pbest[i].plu);

            //Reduce the plume strength over time
            pbest[i].plu = pbest[i].plu * (1-g.pso_reduce);

            if (X[i].plu > g.pso_source_found) {
                //If any of the blimps have found the source, finish source localisation.
                set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Blimp number %.0f found source. Finished.", i+1.0);
            }
            AP::logger().WriteStreaming("PSOP", "TimeUS,i,plu,x,y", "s#---", "F----",
                        "QBfff", AP_HAL::micros64(),
                        i, pbest[i].plu, pbest[i].x, pbest[i].y);
        }
        gcs().send_named_float("GBEST", gbest+1);
        AP::logger().WriteStreaming("PSOG", "TimeUS,plu,x,y", "s---", "F---",
                    "Qfff", AP_HAL::micros64(),
                    pbest[gbest].plu, pbest[gbest].x, pbest[gbest].y);
    }
}

float ModePSO::distance(int part1, int part2){
    return sqrtf(powf((X[part1].x - X[part2].x),2) + powf((X[part1].y - X[part2].y),2));
}

//This is called by Blimp's main GCS mavlink whenever any mavlink message comes in.
void ModePSO::handle_msg(const mavlink_message_t &msg)
{
    // decode global-position-int message
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);

        //This is set up for blimps with sysids starting at 1, and put into array with index starting at 0.
        if (msg.sysid <= PAR_MAX && msg.sysid > 0) {
            // curarr[msg.sysid-1] = packet;
            Location part{packet.lat, packet.lon, packet.alt/10, Location::AltFrame::ABSOLUTE}; //divide alt by 10 because mavlink is in mm, Location is in cm
            Vector2f pos;
            if (part.get_vector_xy_from_origin_NE(pos)){
                X[msg.sysid-1].x = pos.x/100; //The above function returns the position in cm...
                X[msg.sysid-1].y = pos.y/100;
                X[msg.sysid-1].time_boot_ms_pos = packet.time_boot_ms;
                new_pos_recd = true;
                // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Got pos %d %f %f %d %d", msg.sysid, pos.x/100, pos.y/100, part.lat, part.lng);
            }
            // else GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Get location for X failed for %d", msg.sysid);
        }
        else {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Received sysid outside range: %d %d %d %f", msg.sysid, int(packet.lat), int(packet.lon), packet.hdg/100.0);
        }
    }
    else if (msg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT) {
        mavlink_named_value_float_t packet;
        mavlink_msg_named_value_float_decode(&msg, &packet);
        //Annoying, but this is how I had to match it...
        if (packet.name[0] == 'P' && packet.name[1] == 'L' && packet.name[2] == 'U' && packet.name[3] == 'S') {
            if (msg.sysid > max_seen) max_seen = msg.sysid;
            //This is actually what each blimp sends out (see Blimp::handle_plume_str()), not the plume strength sent to each one from the GCS, hence not needing to look at target system
            //This is set up for blimps with sysids starting at 1, and put into array with index starting at 0.
            X[msg.sysid-1].plu = packet.value;
            X[msg.sysid-1].time_boot_ms_plu = packet.time_boot_ms;
            if (msg.sysid <= PAR_MAX && msg.sysid > 0) {
                X[msg.sysid-1].time_boot_ms_plu = packet.time_boot_ms;
                X[msg.sysid-1].plu = packet.value;
                new_plu_recd = true;
                if (packet.value > pbest[msg.sysid-1].plu) { //Only update pbest when new strength is higher than old.
                    //Currently just uses the most recent location for that blimp...
                    // pbest[msg.sysid-1].lat = curarr[msg.sysid-1].lat;
                    // pbest[msg.sysid-1].lon = curarr[msg.sysid-1].lon;
                    pbest[msg.sysid-1].x = X[msg.sysid-1].x;
                    pbest[msg.sysid-1].y = X[msg.sysid-1].y;
                    pbest[msg.sysid-1].time_boot_ms_pos = X[msg.sysid-1].time_boot_ms_pos;
                    pbest[msg.sysid-1].plu = packet.value;
                    pbest[msg.sysid-1].time_boot_ms_plu = packet.time_boot_ms;
                    // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Updated pbest for %d, to %f %f", msg.sysid, pbest[msg.sysid-1].x, pbest[msg.sysid-1].y);
                    if (packet.value > pbest[gbest].plu) {
                        // if(gbest != msg.sysid-1) GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Updated gbest to %d.", msg.sysid);
                        gbest = msg.sysid-1;
                    }
                }
            }
            else {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Received sysid outside range: %d %f", msg.sysid, packet.value);
            }
        }
    }
}