#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

bool ModePSO::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();

    return true;
}

#define cc1 0.1   //personal best weighting
#define cc2 0.1   //global best weighting
#define w 0.8     //current velocity weighting
#define speed_limit 0.1
#define min_d 0.5
#define d 0.5     //separation weighting
#define self int(g.sysid_this_mav)
//X is always synced with most recent plume strength's position, strength, and time for both messages.
//Note that all positions are relative to the current blimp's position.


void send_debug_vector(const char *name, Vector3f value)
{
    mavlink_debug_vect_t packet {};
    packet.time_usec = AP_HAL::micros();
    packet.x = value.x;
    packet.y = value.y;
    packet.z = value.z;
    memcpy(packet.name, name, MIN(strlen(name), (uint8_t)MAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN));

    gcs().send_to_active_channels(MAVLINK_MSG_ID_DEBUG_VECT,
                                  (const char *)&packet);
}


//Runs the main loiter controller
void ModePSO::run()
{
    Vector2f A[PAR_MAX];
    float moveX = 0.0;
    float moveY = 0.0;

    for (uint8_t i=0; i<PAR_MAX; i++){
        for (uint8_t j=0; j<PAR_MAX; j++){
            if (i == j) continue;
            if (dist(i,j) < min_d){
                // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"Blimp numbers %d and %d are within min_d.", i+1, j+1);
                moveX += (pbest[i].x - pbest[j].x);
                moveY += (pbest[i].y - pbest[j].y);
            }
        }
        A[i].x += moveX;
        A[i].y += moveY;
        AP::logger().WriteStreaming("PSOA", "TimeUS,moveX,moveY,i", "Qfff",
                                AP_HAL::micros64(),
                                moveX, moveY, (float)i);

    }
    float rr1 = (float)rand()/RAND_MAX;
    float rr2 = (float)rand()/RAND_MAX;
    float rr3 = (float)rand()/RAND_MAX;
    for (int i=0; i<PAR_MAX; i++){
        V[i].x = w*V[i].x + cc1*rr1*(pbest[i].x - X[i].x) + cc2*rr2*(pbest[gbest].x - X[i].x) + d*rr3*A[i].x;
        V[i].y = w*V[i].y + cc1*rr1*(pbest[i].y - X[i].y) + cc2*rr2*(pbest[gbest].y - X[i].y) + d*rr3*A[i].y;
        V[i].x = constrain_float(V[i].x,-speed_limit,speed_limit);
        V[i].y = constrain_float(V[i].y,-speed_limit,speed_limit);

        AP::logger().WriteStreaming("PSOI", "TimeUS,i,Xx,Xy,Vx,Vy,Ax,Ay,px,py,gx,gy,r1,r2", "s#------------", "F-------------",
                                    "QBffffffffffff",
                                    AP_HAL::micros64(),
                                    i,X[i].x, X[i].y, V[i].x, V[i].y, A[i].x, A[i].y, pbest[i].x, pbest[i].y, pbest[gbest].x, pbest[gbest].y, rr1, rr2);
        AP::logger().WriteStreaming("PSOX", "TimeUS,i,x,y,tpos,plu,tplu", "s#-----", "F------",
                                    "QBffifi",
                                    AP_HAL::micros64(),
                                    i,X[i].x, X[i].y, X[i].time_boot_ms_pos, X[i].plu, X[i].time_boot_ms_plu);
    }
    if (AP_HAL::millis() % 1000 < 15) { //Display approx. once per second only
        // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PBest plu: %f, %f, %f, %f, %f, Gbest: %d, self is: %d", pbest[0].plu, pbest[1].plu, pbest[2].plu, pbest[3].plu, pbest[4].plu, gbest, int(g.sysid_this_mav));
        //GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Curr (V), then 2: %d %f %f %f %f", self, V[self-1].x, V[self-1].y, V[2].x, V[2].y);
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
}

float ModePSO::dist(int part1, int part2){
    return sqrtf(powf((pbest[part1].x - pbest[part2].x),2) + powf((pbest[part1].y - pbest[part2].y),2));
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
            curarr[msg.sysid-1] = packet;
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
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Updated pbest for %d, to %f %f", msg.sysid, pbest[msg.sysid-1].x, pbest[msg.sysid-1].y);
                    // Location pbestLatLng{Vector3f{pbest[i].x,pbest[i].y, 0},Location::AltFrame::ABOVE_ORIGIN};
                    send_debug_vector("PBEST", Vector3f{pbest[msg.sysid-1].x, pbest[msg.sysid-1].y, float(msg.sysid)});
                    if (packet.value > pbest[gbest].plu) {
                        if(gbest != msg.sysid-1) GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Updated gbest to %d.", msg.sysid);
                        gcs().send_named_float("GBEST", msg.sysid);
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