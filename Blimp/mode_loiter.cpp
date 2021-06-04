#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */


 bool ModeLoiter::init(bool ignore_checks){
    target_pos = blimp.position_ned;
    target_yaw = blimp.vel_yaw_filt;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MIR init loiter: %f %f %f %f", target_pos.x, target_pos.y, target_pos.z, target_yaw);
    return true;
 }

//Runs the main loiter controller
void ModeLoiter::run()
{
    float yaw_ef = blimp.ahrs.get_yaw(); //TODO Double-check this function

    //TODO Perhaps put a check in here to ensure that the target doesn't get too far from the vehicle.
    float pilot_fwd = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_pos;
    float pilot_rgt = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_pos;
    float pilot_dwn = channel_down->get_control_in()  / float(RC_SCALE) * g.max_xy_pos;
    float pilot_yaw = channel_yaw->get_control_in() / float(RC_SCALE) * g.max_yaw_pos; 
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
    Vector3f target_vel_ef{blimp.pid_pos_xy.update_all(target_pos, blimp.position_ned), 0};
    target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, blimp.position_ned.z);
    float target_vel_yaw = blimp.pid_pos_yaw.update_all(target_yaw, yaw_ef);

    Vector3f vel_bf = ahrs.get_rotation_body_to_ned().transposed() * blimp.velocity_ned_filt;

    //TODO Should this be a 2D conversion instead?
    Vector3f target_vel_bf = ahrs.get_rotation_body_to_ned().transposed() * target_vel_ef;
    Vector3f target_vel_bf_c{constrain_float(target_vel_bf.x, -g.max_xy_vel, g.max_xy_vel),
                              constrain_float(target_vel_bf.y, -g.max_xy_vel, g.max_xy_vel),
                              constrain_float(target_vel_bf.z, -g.max_xy_vel, g.max_xy_vel)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -g.max_yaw_vel, g.max_yaw_vel);                                

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel_bf_c, vel_bf);
    float act_down = blimp.pid_vel_z.update_all(target_vel_bf_c.z, vel_bf.z);
    float act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c, blimp.vel_yaw_filt);

    if(!(blimp.g.dis_mask & (1<<(2-1)))){
    motors->front_out = actuator.x;
    } if(!(blimp.g.dis_mask & (1<<(1-1)))){
    motors->right_out = actuator.y;
    } if(!(blimp.g.dis_mask & (1<<(3-1)))){
    motors->down_out = act_down;
    } if(!(blimp.g.dis_mask & (1<<(4-1)))){
    motors->yaw_out  = act_yaw;
    }

    AP::logger().Write_PSC(target_pos*100.0f, blimp.position_ned*100.0f, target_vel_bf_c*100.0f, vel_bf*100.0f, blimp.velocity_ned, 0, channel_down->get_control_in()*100.0f); //Last entry is just for debugging rc failsafe issue
    AP::logger().Write_PSCZ(target_pos.z*100.0f, blimp.position_ned.z*100.0f, 0.0f, target_vel_bf_c.z*100.0f, vel_bf.z*100.0f, 0.0f, blimp.velocity_ned.z, blimp.vel_yaw*100.0f, blimp.vel_yaw_filt*100.0f);
    AP::logger().Write_PID(LOG_PIDN_MSG, blimp.pid_vel_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDE_MSG, blimp.pid_vel_xy.get_pid_info_y());
    AP::logger().Write_PID(LOG_PIDR_MSG, blimp.pid_pos_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDP_MSG, blimp.pid_pos_xy.get_pid_info_y());
}
