#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */

class Boid
{
public:
    int32_t   lat;
    int32_t   lon;
    float     hdg;
    float     plu;
    //packet also has alt and velocitiy in x,y,z directions

    constexpr Boid()
        : lat(0)
        , lon(0)
        , hdg(0.0)
        , plu(0.0) {}
};

#define BOID_MAX 5
//This should always containg the most recently received info for each blimp
Boid boidarr[BOID_MAX];

bool ModeBoids::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();

    return true;
}

//Runs the main loiter controller
void ModeBoids::run()
{
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});

    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Curr hdg: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", boidarr[0].hdg, boidarr[1].hdg, boidarr[2].hdg, boidarr[3].hdg, boidarr[4].hdg);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Curr plu: %f, %f, %f, %f, %f", boidarr[0].plu, boidarr[1].plu, boidarr[2].plu, boidarr[3].plu, boidarr[4].plu);
}

//This is called by Blimp's main GCS mavlink whenever any mavlink message comes in.
void ModeBoids::handle_msg(const mavlink_message_t &msg)
{
    // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Boids: Received mavlink msg");
    // skip our own messages
    // if (msg.sysid == mavlink_system.sysid) {
    //     return;
    // }

    // decode global-position-int message
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);

        //packet gives heading in centidegrees, so convert.
        // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Boids: Received mavlink pos: %d %d %d %d %f", mavlink_system.sysid, msg.sysid, packet.lat, packet.lon, packet.hdg/100.0);

        //This is set up for blimps with sysids starting at 1, and put into array with index starting at 0.
        if (msg.sysid <= BOID_MAX && msg.sysid > 0) {
            boidarr[msg.sysid-1].lat = packet.lat;
            boidarr[msg.sysid-1].lon = packet.lon;
            boidarr[msg.sysid-1].hdg = packet.hdg/100.0;
        }
        else {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Boids: Received sysid outside range: %d %d %d %f", msg.sysid, packet.lat, packet.lon, packet.hdg/100.0);
        }
    }
    else if (msg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT) {
        mavlink_named_value_float_t packet;
        mavlink_msg_named_value_float_decode(&msg, &packet);
        //Annoying, but this is how I had to match it...
        if (packet.name[0] == 'P' && packet.name[1] == 'L' && packet.name[2] == 'U' && packet.name[3] == 'S') {
            //This is set up for blimps with sysids starting at 1, and put into array with index starting at 0.
            if (msg.sysid <= BOID_MAX && msg.sysid > 0) {
                boidarr[msg.sysid-1].plu = packet.value;
            }
            else {
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Boids: Received sysid outside range: %d %f", msg.sysid, packet.value);
            }
        }
    }
}
