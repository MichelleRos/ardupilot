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
    //packet also has alt and velocitiy in x,y,z directions

    constexpr Boid()
        : lat(0)
        , lon(0)
        , hdg(0.0) {}

    constexpr Boid(const int32_t lat0, const int32_t lon0, const float hdg0)
        : lat(lat0)
        , lon(lon0)
        , hdg(hdg0) {} //must convert from int cdeg to float deg before putting it in.
};

Boid boidarr[5];

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
}

void ModeBoids::set_target(float angle) //angle is in radians
{
    // Vector3f tar_vec;
    // if(tar.get_vector_from_origin_NEU(tar_vec)){ //Unfortunately returns vector in cm instead of m...
    //     target_pos.x = tar_vec.x/100.0 + g.guid_ofs_x;
    //     target_pos.y = tar_vec.y/100.0 + g.guid_ofs_y;
    //     //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Received target: %f, %f", target_pos.x, target_pos.y);
    // }
    Vector3f target = {0.2,0.0,0.0};
    target.rotate_xy(angle);
    target_yaw = angle;
    blimp.rotate_NE_to_BF(target.xy());
    target_pos = target_pos + target;
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
        boidarr[msg.sysid-1].lat = packet.lat;
        boidarr[msg.sysid-1].lon = packet.lon;
        boidarr[msg.sysid-1].hdg = packet.hdg/100.0;

    }
}
