#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */


 bool ModeLoiter::init(bool ignore_checks){
    Vector3f pos_ef;
    bool gps_avail = blimp.ahrs.get_relative_position_NED_home(pos_ef);
        if (!gps_avail) {
        //Shouldn't reach this since it should failsafe into Land mode.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }

    target_pos.x = pos_ef.x;
    target_pos.y = pos_ef.y;
    target_pos.z = pos_ef.z;
    target_yaw = blimp.ahrs.get_yaw(); //TODO Double-check this function

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MIR: Initted Loiter.");
    return true;
 }

//Runs the main loiter controller
void ModeLoiter::run()
{
    Vector3f pos_ef;
    bool gps_avail = blimp.ahrs.get_relative_position_NED_home(pos_ef);
        if (!gps_avail) {
        //Shouldn't reach this since it should failsafe into Land mode.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    float yaw_ef = blimp.ahrs.get_yaw(); //TODO Double-check this function

    //TODO Perhaps put a check in here to ensure that the target doesn't get too far from the vehicle.
    float pilot_fwd = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_pos;
    float pilot_rgt = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_pos;
    float pilot_dwn = channel_down->get_control_in()  / float(RC_SCALE) * g.max_xy_pos;
    float pilot_yaw = channel_yaw->get_control_in() / float(RC_SCALE) * g.max_xy_pos; 
    if (g.simple_mode == 1){
        target_pos.x = target_pos.x + pilot_fwd;
        target_pos.y = target_pos.y + pilot_rgt;
        target_pos.z = target_pos.z + pilot_dwn;
        target_yaw = target_yaw + pilot_yaw;
    } else {
        target_pos.x += (pilot_fwd*blimp.ahrs.cos_yaw() - pilot_rgt*blimp.ahrs.sin_yaw());
        target_pos.y += (pilot_fwd*blimp.ahrs.sin_yaw() + pilot_rgt*blimp.ahrs.cos_yaw());
        target_pos.z += pilot_dwn;
        target_yaw += pilot_yaw;
    }

    //pos controller's output becomes target for velocity controller
    Vector3f target_vel_ef{blimp.pid_pos_xy.update_all(target_pos, pos_ef), 0};
    target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, pos_ef.z);
    float target_vel_yaw = blimp.pid_pos_yaw.update_all(target_yaw, yaw_ef);

    Vector3f vel_ef;
    gps_avail = ahrs.get_velocity_NED(vel_ef); //earth-frame velocity
    if (!gps_avail) {
        //Shouldn't reach this since it should failsafe into Land mode.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    float vel_yaw = blimp.ahrs.get_yaw_rate_earth();

    //TODO Should this be a 2D conversion instead?
    Vector3f target_vel_bf = ahrs.get_rotation_body_to_ned().transposed() * target_vel_ef;
    Vector3f vel_bf = ahrs.get_rotation_body_to_ned().transposed() * vel_ef;
    Vector3f vel_bf_xy{constrain_float(vel_bf.x, -g.max_xy_vel, g.max_xy_vel),
                                    constrain_float(vel_bf.y, -g.max_xy_vel, g.max_xy_vel),
                                    0};
    vel_bf_xy.z = constrain_float(vel_bf.z, -g.max_xy_vel, g.max_xy_vel);
    float vel_yaw_c = constrain_float(vel_yaw, -g.max_xy_vel, g.max_xy_vel);                                

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel_bf, vel_bf_xy);
    float down = blimp.pid_vel_z.update_all(target_vel_bf.z, vel_bf_xy.z);
    float yaw = blimp.pid_vel_z.update_all(target_vel_yaw, vel_yaw_c);

    motors->right_out = actuator.y;
    motors->front_out = actuator.x;

    motors->yaw_out = yaw;
    motors->down_out = down;

    AP::logger().Write_PSC(target_pos*100.0f, pos_ef*100.0f, target_vel_bf*100.0f, vel_bf_xy*100.0f, {0,0,0}, 0, 0);
    AP::logger().Write_PID(LOG_PIDN_MSG, blimp.pid_vel_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDE_MSG, blimp.pid_vel_xy.get_pid_info_y());
    AP::logger().Write_PID(LOG_PIDR_MSG, blimp.pid_pos_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDP_MSG, blimp.pid_pos_xy.get_pid_info_y());
}
