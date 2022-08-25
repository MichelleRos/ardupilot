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

//Runs the main loiter controller
void ModePSO::run()
{
    if (AP_HAL::millis() % 1000 < 15) { //Display approx. once per second only
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PBest plu: %f, %f, %f, %f, %f, Gbest: %d", pbest[0].plu, pbest[1].plu, pbest[2].plu, pbest[3].plu, pbest[4].plu, gbest);
    }

    Vector2f A[PAR_MAX];
    float moveX = 0.0;
    float moveY = 0.0;

    for (uint8_t i=0; i<PAR_MAX; i++){
        for (uint8_t j=0; j<PAR_MAX; j++){
            if (i == j) continue;
            if (dist(i,j) < min_d){
                moveX += (pbest[i].pn - pbest[j].pn);
                moveY += (pbest[i].pe - pbest[j].pe);
            }
        }
        A[i].x += moveX;
        A[i].y += moveY;
        AP::logger().WriteStreaming("PSO1", "TimeUS,moveX,moveY,i",
                                "Qfff",
                                AP_HAL::micros64(),
                                moveX, moveY, (float)i);
    }
    float rr1 = rand()/RAND_MAX;
    float rr2 = rand()/RAND_MAX;
    for (int i=0; i<PAR_MAX; i++){
        V[i].x = w*V[i].x + d*A[i].x + cc1*rr1*(pbest[i].pn - X[i].x) - cc2*rr2*(pbest[gbest].pn - X[i].x);
        V[i].y = w*V[i].y + d*A[i].y + cc1*rr1*(pbest[i].pe - X[i].y) - cc2*rr2*(pbest[gbest].pe - X[i].y);
        if (V[i].x > speed_limit) V[i].x = speed_limit;
        if (V[i].y > speed_limit) V[i].y = speed_limit;
        X[i].x = X[i].x + V[i].x*blimp.scheduler.get_loop_period_s();
        X[i].y = X[i].y + V[i].y*blimp.scheduler.get_loop_period_s();
    }

    target_pos = {X[g.sysid_this_mav].x,X[g.sysid_this_mav].y,0};
    target_yaw = 0;
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}

float ModePSO::dist(int part1, int part2){
    // return math.sqrt((part1[0] - particle2[0])**2 + (particle1[1] - particle2[1])**2)
    return sqrtf(powf((pbest[part1].pn - pbest[part2].pn),2) + powf((pbest[part1].pe - pbest[part2].pe),2));
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
            Location part{curarr[msg.sysid-1].lat, curarr[msg.sysid-1].lon, 0, Location::AltFrame::ABOVE_ORIGIN};
            Vector2f pos;
            if (part.get_vector_xy_from_origin_NE(pos)){
                X[msg.sysid-1].x = pos.x;
                X[msg.sysid-1].y = pos.y;
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
            //This is set up for blimps with sysids starting at 1, and put into array with index starting at 0.
            if (msg.sysid <= PAR_MAX && msg.sysid > 0) {
                if (packet.value > pbest[msg.sysid-1].plu) { //Only update pbest when new strength is higher than old.
                    //Currently just uses the most recent location for that blimp...
                    // pbest[msg.sysid-1].lat = curarr[msg.sysid-1].lat;
                    // pbest[msg.sysid-1].lon = curarr[msg.sysid-1].lon;
                    Location part{curarr[msg.sysid-1].lat, curarr[msg.sysid-1].lon, 0, Location::AltFrame::ABOVE_ORIGIN};
                    Vector2f pos;
                    if (part.get_vector_xy_from_origin_NE(pos)){
                        pbest[msg.sysid-1].pn = pos.x;
                        pbest[msg.sysid-1].pe = pos.y;
                    } else GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Get location for pbest failed for %d", msg.sysid);
                    pbest[msg.sysid-1].plu = packet.value;
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Updated pbest for %d", msg.sysid);
                    if (packet.value > pbest[gbest].plu) {
                        gbest = msg.sysid;
                        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Updated gbest.");
                    }
                }
            }
            else {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PSO: Received sysid outside range: %d %f", msg.sysid, packet.value);
            }
        }
    }
}