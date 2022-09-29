#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

bool ModePSO::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();
    gbest = 1;
    for (uint8_t i=0; i<PAR_MAX; i++) V[i].zero();
    for (uint8_t i=0; i<PAR_MAX; i++) X[i].zero();
    for (uint8_t i=0; i<PAR_MAX; i++) pbest[i].zero();
    return true;
}

#define self int(g.sysid_this_mav)
//Using this "ran" macro so that random number is regenerated for each new use
#define ran (float)rand()/RAND_MAX
//X is always synced with most recent plume strength's position, strength, and time for both messages.
//Note that all positions are relative to the current blimp's home.


void send_debug_loc(const char *name, Location value)
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

//Runs the main loiter controller
void ModePSO::run()
{
    Vector2f A[PAR_MAX];
    float moveX = 0.0;
    float moveY = 0.0;

    for (uint8_t i=0; i<max_seen; i++){
        AP::logger().WriteStreaming("PSOB", "TimeUS,i,moveX,moveY", "s#--", "F---","QBff",
                                AP_HAL::micros64(),
                                i, moveX, moveY);
        for (uint8_t j=0; j<max_seen; j++){
            if (i == j) continue;
            if (dist(i,j) < g.pso_min_dist){
                // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"Blimp numbers %d and %d are within min_d.", i+1, j+1);
                moveX += (X[i].x - X[j].x);
                moveY += (X[i].y - X[j].y);
                AP::logger().WriteStreaming("PSOD", "TimeUS,i,j,dist", "s#--", "F---","QBBf",
                                            AP_HAL::micros64(),
                                            i, j, dist(i,j));
            }
        }
        A[i].x += moveX;
        A[i].y += moveY;
        AP::logger().WriteStreaming("PSOA", "TimeUS,i,moveX,moveY", "s#--", "F---","QBff",
                                AP_HAL::micros64(),
                                i, moveX, moveY);
    }
    for (int i=0; i<max_seen; i++){
        V[i].x = g.pso_w_vel*V[i].x + g.pso_w_per_best*ran*(pbest[i].x - X[i].x) + g.pso_w_glo_best*ran*(pbest[gbest].x - X[i].x) + g.pso_w_avoid*ran*A[i].x;
        V[i].y = g.pso_w_vel*V[i].y + g.pso_w_per_best*ran*(pbest[i].y - X[i].y) + g.pso_w_glo_best*ran*(pbest[gbest].y - X[i].y) + g.pso_w_avoid*ran*A[i].y;
        V[i].x = constrain_float(V[i].x,-g.pso_speed_limit,g.pso_speed_limit);
        V[i].y = constrain_float(V[i].y,-g.pso_speed_limit,g.pso_speed_limit);

        AP::logger().WriteStreaming("PSOI", "TimeUS,i,Xx,Xy,Vx,Vy,Ax,Ay,px,py,gx,gy,r1,r2", "s#------------", "F-------------",
                                    "QBffffffffffff",
                                    AP_HAL::micros64(),
                                    i,X[i].x, X[i].y, V[i].x, V[i].y, A[i].x, A[i].y, pbest[i].x, pbest[i].y, pbest[gbest].x, pbest[gbest].y, 0.0, 0.0);
        AP::logger().WriteStreaming("PSOX", "TimeUS,i,x,y,tpos,plu,tplu", "s#-----", "F------",
                                    "QBffIfI",
                                    AP_HAL::micros64(),
                                    i,X[i].x, X[i].y, X[i].time_boot_ms_pos, X[i].plu, X[i].time_boot_ms_plu);
    }

    Vector3f target_vel = {V[self-1].x,V[self-1].y,0};
    //target_pos = {blimp.pos_ned.x = V[g.sysid_this_mav].x*blimp.scheduler.get_loop_period_s(), blimp.pos_ned.y + V[g.sysid_this_mav].y*blimp.scheduler.get_loop_period_s(), 0};

    //Don't let target pos get further away than 1m
    // if(fabsf(target_pos.x-blimp.pos_ned.x) < 1.0f) target_pos.x += V[self-1].x;
    // if(fabsf(target_pos.y-blimp.pos_ned.y) < 1.0f) target_pos.y += V[self-1].y;
    target_yaw = 0;
    blimp.loiter->run_vel(target_vel, target_yaw, Vector4b{false,false,false,false});
    AP::logger().WriteStreaming("PSOT", "TimeUS,tvx,tvy", "Qff",
                                    AP_HAL::micros64(),
                                    target_vel.x, target_vel.y);

    uint32_t now = AP_HAL::millis();
    if((now - bests_sent) > 1000){
        bests_sent = now;
        //For some mad reason, this x and y is in cm
        for (int i=0; i<max_seen; i++){
            Location pbestLatLng{Vector3f{pbest[i].x*100,pbest[i].y*100, 0.0f},Location::AltFrame::ABSOLUTE};
            char nm[10];
            hal.util->snprintf(nm, sizeof(nm), "PBEST%d", i+1);
            send_debug_loc(nm, pbestLatLng);

            //Reduce the plume strength over time
            pbest[i].plu = pbest[i].plu * (1-g.pso_reduce);
        }
        gcs().send_named_float("GBEST", gbest+1);
    }
}

float ModePSO::dist(int part1, int part2){
    // return sqrtf(powf((pbest[part1].x - pbest[part2].x),2) + powf((pbest[part1].y - pbest[part2].y),2));
    Vector3f p1{X[part1].x, X[part1].y, 0};
    Vector3f p2{X[part2].x, X[part2].y, 0};
    return sqrtf(p1.distance_squared(p2));
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
                // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Got pos %d %f %f %d %d", msg.sysid, pos.x/100, pos.y/100, part.lat, part.lng);
            } else GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Get location for X failed for %d", msg.sysid);
        }
        else {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Received sysid outside range: %d %d %d %f", msg.sysid, packet.lat, packet.lon, packet.hdg/100.0);
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